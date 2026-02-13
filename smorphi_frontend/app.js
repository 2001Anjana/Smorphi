const PI_IP = "10.0.0.6";
let socket = null;

function log(message) {
  const logBox = document.getElementById("logBox");
  logBox.textContent += message + "\n";
  logBox.scrollTop = logBox.scrollHeight;
}

// ---------------- CONNECT ----------------

function connectRos() {
  fetch(`http://${PI_IP}:8000/health`)
    .then(res => res.json())
    .then(() => {
      document.getElementById("connectionStatus").textContent = "Connected";
      document.getElementById("connectionStatus").style.color = "green";
      connectLogs();
    })
    .catch(() => {
      document.getElementById("connectionStatus").textContent = "Failed";
      document.getElementById("connectionStatus").style.color = "red";
    });
}

function connectLogs() {
  socket = new WebSocket(`ws://${PI_IP}:8000/ws/logs`);
  socket.onmessage = (event) => log(event.data);
  socket.onopen = () => {
    console.log("WebSocket connected");
    socket.send("ls"); 
  };
  socket.onclose = () => log("Log stream closed");
}

function sendCommand(cmd) {
  if (socket) {
    socket.send(cmd);
  } else {
    log("WebSocket not connected");
  }
}

// ---------------- BUTTON COMMANDS ----------------

// 1️⃣ Build workspace (Confirmed Working)
function build() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && colcon build"');
}

// 2️⃣ Bringup (Confirmed Working)
function bringupStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py"');
}

function bringupStop() {
  sendCommand('bash -c "pkill -f smorphi_bringup.launch.py"');
}

// 3️⃣ Teleop - Corrected to use the keyboard node from README
function teleopEnable() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"');
}

// 4️⃣ RViz - Standard launch
function rvizStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && rviz2"');
}

// 5️⃣ & 6️⃣ - These nodes (auto_transform/mpu_odom) aren't in your README 
// but assuming they are part of your custom 'smorphi' package:
function transformEnable() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 run smorphi auto_transform_node --enable"');
}

// 7️⃣ Mapping - Corrected to "smorphi_mapper_online_async_launch.py" from README
function mappingStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 launch smorphi_ros2_launchers smorphi_mapper_online_async_launch.py"');
}

function mappingStop() {
  sendCommand('bash -c "pkill -f smorphi_mapper_online_async_launch.py"');
}

// 8️⃣ Navigation - Corrected to "smorphi_nav2.launch.py" from README
function navStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 launch smorphi_ros2_launchers smorphi_nav2.launch.py"');
}

function navStop() {
  sendCommand('bash -c "pkill -f smorphi_nav2.launch.py"');
}

// 9️⃣ Map Saver - Added based on Activity 2 in README
function saveMap(mapName = "my_map") {
  sendCommand(`bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ${mapName}"`);
}