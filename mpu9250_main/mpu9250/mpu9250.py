import os
import sys
import time
import smbus
import numpy as np
import threading
from collections import deque

from .imusensor.MPU9250 import MPU9250
from .imusensor.filters import kalman 
from .imusensor.filters import madgwick

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped

from math import sin, cos, radians, degrees, atan2, sqrt
import tf_transformations


class MyPythonNode(Node):
    def __init__(self):
        super().__init__("mpu9250")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 30),
                ('frame_id', 'imu_link'),
                ('i2c_address', 0x68),
                ('i2c_port', 1),
                ('acceleration_scale', [1.0, 1.0, 1.0]),
                ('acceleration_bias', [0.0, 0.0, 0.0]),
                ('gyro_bias', [0.0, 0.0, 0.0]),
                ('magnetometer_scale', [1.0, 1.0, 1.0]),
                ('magnetometer_bias', [0.0, 0.0, 0.0]),
                ('magnetometer_transform', [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]),
                ('use_magnetometer', True),
                ('mag_timeout_ms', 100),
                ('mag_filter_alpha', 0.1),  # Low-pass filter for magnetometer
                ('complementary_alpha', 0.98),  # Complementary filter coefficient
                ('publish_raw_mag', True),  # Publish raw magnetometer data
                ('mag_validity_threshold', 5.0),  # Minimum magnetic field magnitude
                ('debug_output', True),  # Enable debug output
            ]
        )

        # Initialize I2C and IMU
        address = self.get_parameter('i2c_address')._value
        bus = smbus.SMBus(self.get_parameter('i2c_port')._value)
        self.imu = MPU9250.MPU9250(bus, address)

        # Store references
        self.bus = bus
        self.address = address
        self.use_magnetometer = self.get_parameter('use_magnetometer')._value
        self.mag_timeout_ms = self.get_parameter('mag_timeout_ms')._value
        self.mag_filter_alpha = self.get_parameter('mag_filter_alpha')._value
        self.complementary_alpha = self.get_parameter('complementary_alpha')._value
        self.publish_raw_mag = self.get_parameter('publish_raw_mag')._value
        self.mag_validity_threshold = self.get_parameter('mag_validity_threshold')._value
        self.debug_output = self.get_parameter('debug_output')._value

        # Magnetometer state
        self.mag_available = False
        self.mag_initialized = False
        self.mag_error_count = 0
        self.mag_consecutive_errors = 0
        self.last_good_mag_time = time.time()
        self.mag_cal_x = self.mag_cal_y = self.mag_cal_z = 1.0
        
        # Magnetometer filtering
        self.mag_filtered = np.array([0.0, 0.0, 0.0])
        self.mag_history = deque(maxlen=5)  # Keep last 5 readings for validation
        
        # Thread safety
        self.mag_lock = threading.Lock()
        
        # IMU calibration parameters
        self.imu.Accels = np.asarray(self.get_parameter('acceleration_scale')._value)
        self.imu.AccelBias = np.asarray(self.get_parameter('acceleration_bias')._value)
        self.imu.GyroBias = np.asarray(self.get_parameter('gyro_bias')._value)
        self.imu.Mags = np.asarray(self.get_parameter('magnetometer_scale')._value)
        self.imu.MagBias = np.asarray(self.get_parameter('magnetometer_bias')._value)
        self.imu.Magtransform = np.reshape(np.asarray(self.get_parameter('magnetometer_transform')._value),(3,3))

        # Publishers
        self.publisher_imu_values_ = self.create_publisher(Imu, "/imu", 10)
        if self.publish_raw_mag:
            self.publisher_mag_values_ = self.create_publisher(MagneticField, "/imu/mag", 10)
        
        # Timer for publishing
        self.timer_publish_imu_values_ = self.create_timer(
            1.0/self.get_parameter('frequency')._value, self.publish_imu_values)

        # Sensor fusion
        self.sensorfusion = kalman.Kalman()
        # Alternative: self.sensorfusion = madgwick.Madgwick(0.1)
        
        # Initialize magnetometer if enabled
        if self.use_magnetometer:
            self.initialize_magnetometer()
        else:
            self.get_logger().info("Magnetometer disabled by parameter")

        # Initialize IMU
        self.imu.begin()
        time.sleep(0.1)
        self.imu.readSensor()
        
        # Initialize orientation estimation
        self.initialize_orientation()
        
        # Timing
        self.deltaTime = 0
        self.lastTime = self.get_clock().now()
        
        # Statistics
        self.sample_count = 0
        self.mag_valid_count = 0
        self.last_stats_time = time.time()

    def initialize_magnetometer(self):
        """Initialize magnetometer with improved error handling"""
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self.get_logger().info(f"Initializing magnetometer (attempt {attempt + 1}/{max_attempts})")
                
                # MPU9250 register addresses
                PWR_MGMT_1 = 0x6B
                USER_CTRL = 0x6A
                INT_PIN_CFG = 0x37
                
                # Wake up MPU9250
                self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
                time.sleep(0.1)
                
                # Disable I2C master mode
                self.bus.write_byte_data(self.address, USER_CTRL, 0x00)
                time.sleep(0.1)
                
                # Enable I2C bypass mode
                self.bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
                time.sleep(0.1)
                
                # Initialize AK8963
                if self.setup_ak8963():
                    self.mag_available = True
                    self.mag_initialized = True
                    self.get_logger().info("Magnetometer initialized successfully")
                    return
                else:
                    self.get_logger().warn(f"AK8963 setup failed on attempt {attempt + 1}")
                    
            except Exception as e:
                self.get_logger().error(f"Magnetometer initialization attempt {attempt + 1} failed: {e}")
                time.sleep(0.5)
        
        self.get_logger().error("Failed to initialize magnetometer after all attempts")
        self.mag_available = False
        self.mag_initialized = False

    def setup_ak8963(self):
        """Setup AK8963 magnetometer"""
        try:
            AK8963_ADDRESS = 0x0C
            AK8963_WIA = 0x00
            AK8963_CNTL1 = 0x0A
            AK8963_CNTL2 = 0x0B
            AK8963_ASAX = 0x10
            
            # Check Who Am I
            who_am_i = self.bus.read_byte_data(AK8963_ADDRESS, AK8963_WIA)
            if who_am_i != 0x48:
                self.get_logger().error(f"AK8963 Who Am I failed: got 0x{who_am_i:02x}, expected 0x48")
                return False
            
            # Reset AK8963
            self.bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL2, 0x01)
            time.sleep(0.1)
            
            # Power down
            self.bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL1, 0x00)
            time.sleep(0.1)
            
            # Enter fuse ROM access mode
            self.bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL1, 0x0F)
            time.sleep(0.1)
            
            # Read calibration data
            cal_data = self.bus.read_i2c_block_data(AK8963_ADDRESS, AK8963_ASAX, 3)
            self.mag_cal_x = (cal_data[0] - 128) / 256.0 + 1.0
            self.mag_cal_y = (cal_data[1] - 128) / 256.0 + 1.0
            self.mag_cal_z = (cal_data[2] - 128) / 256.0 + 1.0
            
            self.get_logger().info(f"Magnetometer calibration: X={self.mag_cal_x:.3f}, Y={self.mag_cal_y:.3f}, Z={self.mag_cal_z:.3f}")
            
            # Power down
            self.bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL1, 0x00)
            time.sleep(0.1)
            
            # Set to continuous measurement mode (16-bit, 100Hz)
            self.bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL1, 0x16)
            time.sleep(0.1)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"AK8963 setup failed: {e}")
            return False

    def read_magnetometer_with_validation(self):
        """Read magnetometer with improved validation and filtering"""
        if not self.mag_available:
            return np.array([0.0, 0.0, 0.0]), False
        
        try:
            AK8963_ADDRESS = 0x0C
            AK8963_ST1 = 0x02
            AK8963_HXL = 0x03
            AK8963_ST2 = 0x09
            
            # Check data ready with timeout
            data_ready = False
            for _ in range(5):  # Try up to 5 times
                status1 = self.bus.read_byte_data(AK8963_ADDRESS, AK8963_ST1)
                if status1 & 0x01:
                    data_ready = True
                    break
                time.sleep(0.001)  # 1ms delay
            
            if not data_ready:
                return np.array([0.0, 0.0, 0.0]), False
            
            # Read all magnetometer registers at once
            mag_data = self.bus.read_i2c_block_data(AK8963_ADDRESS, AK8963_HXL, 7)
            
            # Check for overflow
            st2 = mag_data[6]
            if st2 & 0x08:  # Overflow bit
                return np.array([0.0, 0.0, 0.0]), False
            
            # Convert to signed 16-bit values
            mag_x = np.int16((mag_data[1] << 8) | mag_data[0])
            mag_y = np.int16((mag_data[3] << 8) | mag_data[2])
            mag_z = np.int16((mag_data[5] << 8) | mag_data[4])
            
            # Apply factory calibration
            mag_x *= self.mag_cal_x
            mag_y *= self.mag_cal_y
            mag_z *= self.mag_cal_z
            
            # Convert to microTesla
            scale_factor = 0.15  # µT/LSB for 16-bit mode
            mag_values = np.array([mag_x * scale_factor, mag_y * scale_factor, mag_z * scale_factor])
            
            # Validate magnetometer reading
            mag_magnitude = np.linalg.norm(mag_values)
            if mag_magnitude < self.mag_validity_threshold or mag_magnitude > 100.0:  # Reasonable range for Earth's magnetic field
                return np.array([0.0, 0.0, 0.0]), False
            
            # Apply low-pass filter
            if hasattr(self, 'mag_filtered'):
                self.mag_filtered = self.mag_filter_alpha * mag_values + (1 - self.mag_filter_alpha) * self.mag_filtered
            else:
                self.mag_filtered = mag_values.copy()
            
            # Update history for trend analysis
            self.mag_history.append(mag_values.copy())
            
            # Reset error counters
            self.mag_consecutive_errors = 0
            self.last_good_mag_time = time.time()
            
            return self.mag_filtered.copy(), True
            
        except Exception as e:
            self.mag_error_count += 1
            self.mag_consecutive_errors += 1
            
            if self.mag_consecutive_errors > 10:
                self.get_logger().warn("Too many consecutive magnetometer errors - reinitializing")
                self.mag_available = False
                self.create_timer(2.0, self.try_reinitialize_magnetometer)
            
            return np.array([0.0, 0.0, 0.0]), False

    def try_reinitialize_magnetometer(self):
        """Try to reinitialize magnetometer after errors"""
        if self.use_magnetometer and not self.mag_available:
            self.get_logger().info("Attempting magnetometer reinitialization...")
            self.mag_consecutive_errors = 0
            self.initialize_magnetometer()

    def initialize_orientation(self):
        """Initialize orientation estimation"""
        try:
            # Take several readings to get stable initial values
            accel_sum = np.zeros(3)
            gyro_sum = np.zeros(3)
            mag_sum = np.zeros(3)
            valid_samples = 0
            
            for _ in range(10):
                self.imu.readSensor()
                accel_sum += self.imu.AccelVals
                gyro_sum += self.imu.GyroVals
                
                if self.mag_available:
                    mag_vals, mag_valid = self.read_magnetometer_with_validation()
                    if mag_valid:
                        mag_sum += mag_vals
                        valid_samples += 1
                
                time.sleep(0.01)
            
            # Calculate initial orientation from accelerometer
            accel_avg = accel_sum / 10.0
            initial_roll = atan2(accel_avg[1], accel_avg[2]) * 180.0 / np.pi
            initial_pitch = atan2(-accel_avg[0], sqrt(accel_avg[1]**2 + accel_avg[2]**2)) * 180.0 / np.pi
            initial_yaw = 0.0
            
            # Calculate initial yaw from magnetometer if available
            if valid_samples > 5:
                mag_avg = mag_sum / valid_samples
                # Simple magnetic declination calculation
                # This is a simplified approach - you may need to adjust based on your location
                initial_yaw = atan2(mag_avg[1], mag_avg[0]) * 180.0 / np.pi
            
            # Initialize sensor fusion
            self.sensorfusion.roll = initial_roll
            self.sensorfusion.pitch = initial_pitch
            self.sensorfusion.yaw = initial_yaw
            
            self.get_logger().info(f"Initial orientation: roll={initial_roll:.1f}°, pitch={initial_pitch:.1f}°, yaw={initial_yaw:.1f}°")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize orientation: {e}")
            self.sensorfusion.roll = 0.0
            self.sensorfusion.pitch = 0.0
            self.sensorfusion.yaw = 0.0

    def publish_imu_values(self):
        """Main publishing function with improved sensor fusion"""
        try:
            # Read IMU sensors
            self.imu.readSensor()
            
            # Read magnetometer
            mag_vals, mag_valid = self.read_magnetometer_with_validation()

	    # >>> ADD THIS BLOCK HERE (apply your calibration to mag_vals) <<<
            if self.mag_available and mag_valid:
                 mag_cal = mag_vals.copy()

            # subtract hard-iron bias
            mag_cal = mag_cal - self.imu.MagBias

            # choose ONE method (recommended: transform)
            # (1) Simple scale+bias:
            # mag_cal = mag_cal * self.imu.Mags

            # (2) Precise ellipsoid transform+bias:
            mag_cal = self.imu.Magtransform.dot(mag_cal)

            # replace mag_vals used by fusion
            mag_vals = mag_cal
            # >>> END BLOCK <<<            
            
            # Update statistics
            self.sample_count += 1
            if mag_valid:
                self.mag_valid_count += 1
            
            # Calculate time delta
            current_time = self.get_clock().now()
            deltaTime = (current_time - self.lastTime).nanoseconds * 1e-9
            self.lastTime = current_time
            
            # Sensor fusion
            if self.mag_available and mag_valid:
                # 9DOF fusion with magnetometer
                self.sensorfusion.computeAndUpdateRollPitchYaw(
                    self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                    self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                    mag_vals[0], mag_vals[1], mag_vals[2], deltaTime)
            else:
                # 6DOF fusion (complementary filter)
                self.update_orientation_6dof(deltaTime)
            
            # Create and publish IMU message
            self.publish_imu_message(current_time, mag_vals, mag_valid)
            
            # Publish magnetometer message if enabled
            if self.publish_raw_mag and self.mag_available:
                self.publish_magnetometer_message(current_time, mag_vals, mag_valid)
            
            # Debug output
            if self.debug_output:
                self.print_debug_info(mag_vals, mag_valid)
                
        except Exception as e:
            self.get_logger().error(f"Error in publish_imu_values: {e}")

    def update_orientation_6dof(self, deltaTime):
        """6DOF orientation update using complementary filter"""
        # Calculate orientation from accelerometer
        accel_roll = atan2(self.imu.AccelVals[1], self.imu.AccelVals[2]) * 180.0 / np.pi
        accel_pitch = atan2(-self.imu.AccelVals[0], 
                           sqrt(self.imu.AccelVals[1]**2 + self.imu.AccelVals[2]**2)) * 180.0 / np.pi
        
        # Integrate gyroscope
        gyro_roll = self.sensorfusion.roll + self.imu.GyroVals[0] * deltaTime * 180.0 / np.pi
        gyro_pitch = self.sensorfusion.pitch + self.imu.GyroVals[1] * deltaTime * 180.0 / np.pi
        gyro_yaw = self.sensorfusion.yaw + self.imu.GyroVals[2] * deltaTime * 180.0 / np.pi
        
        # Complementary filter
        self.sensorfusion.roll = self.complementary_alpha * gyro_roll + (1 - self.complementary_alpha) * accel_roll
        self.sensorfusion.pitch = self.complementary_alpha * gyro_pitch + (1 - self.complementary_alpha) * accel_pitch
        self.sensorfusion.yaw = gyro_yaw  # Yaw drifts without magnetometer

    def publish_imu_message(self, timestamp, mag_vals, mag_valid):
        """Publish IMU message with proper covariance"""
        msg = Imu()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.get_parameter('frame_id')._value
        
        # Linear acceleration
        msg.linear_acceleration.x = self.imu.AccelVals[0]
        msg.linear_acceleration.y = self.imu.AccelVals[1]
        msg.linear_acceleration.z = self.imu.AccelVals[2]
        msg.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        
        # Angular velocity
        msg.angular_velocity.x = self.imu.GyroVals[0]
        msg.angular_velocity.y = self.imu.GyroVals[1]
        msg.angular_velocity.z = self.imu.GyroVals[2]
        msg.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        
        # Orientation (convert to quaternion)
        quat = tf_transformations.quaternion_from_euler(
            radians(self.sensorfusion.roll), 
            radians(self.sensorfusion.pitch), 
            radians(self.sensorfusion.yaw))
        
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        
        # Orientation covariance (higher uncertainty for yaw without magnetometer)
        if self.mag_available and mag_valid:
            msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        else:
            msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.1]
        
        self.publisher_imu_values_.publish(msg)

    def publish_magnetometer_message(self, timestamp, mag_vals, mag_valid):
        """Publish raw magnetometer data"""
        msg = MagneticField()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.get_parameter('frame_id')._value
        
        # Convert from µT to T
        msg.magnetic_field.x = mag_vals[0] * 1e-6
        msg.magnetic_field.y = mag_vals[1] * 1e-6
        msg.magnetic_field.z = mag_vals[2] * 1e-6
        
        # Covariance (higher if data is not valid)
        if mag_valid:
            covariance = 0.0001
        else:
            covariance = 1.0
        
        msg.magnetic_field_covariance = [covariance, 0.0, 0.0, 0.0, covariance, 0.0, 0.0, 0.0, covariance]
        
        self.publisher_mag_values_.publish(msg)

    def print_debug_info(self, mag_vals, mag_valid):
        """Print debug information"""
        current_time = time.time()
        
        # Print statistics every 5 seconds
        if current_time - self.last_stats_time > 5.0:
            if self.sample_count > 0:
                mag_success_rate = (self.mag_valid_count / self.sample_count) * 100
                self.get_logger().info(f"Magnetometer success rate: {mag_success_rate:.1f}% ({self.mag_valid_count}/{self.sample_count})")
            self.last_stats_time = current_time
            self.sample_count = 0
            self.mag_valid_count = 0
        
        # Print current values
        mag_status = "OK" if self.mag_available else "DISABLED"
        mag_data_status = "VALID" if mag_valid else "INVALID"
        
        print("roll: {:6.2f} \tpitch: {:6.2f} \tyaw: {:6.2f} \tmag: [{:6.2f}, {:6.2f}, {:6.2f}] \tstatus: {} \tdata: {}".format(
            self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw,
            mag_vals[0], mag_vals[1], mag_vals[2], mag_status, mag_data_status))


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
