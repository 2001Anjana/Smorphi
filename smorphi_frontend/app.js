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
    socket.send("ls"); // test command on connect
  };

  socket.onclose = () => log("Log stream closed");
}

// ---------------- STREAMING COMMAND ----------------

function sendCommand(cmd) {
  if (socket) {
    socket.send(cmd);
  } else {
    log("WebSocket not connected");
  }
}

// ---------------- BUTTON COMMANDS ----------------

// 1️⃣ Build workspace
function build() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && colcon build"');
}

// 2️⃣ Bringup
//function bringupStart() {
  //sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py"');
//}

function bringupStart() {
  sendCommand(
    'bash -c "source /opt/ros/humble/setup.bash && cd ~/smorphi_ws && ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py"'
  );
}


function bringupStop() {
  sendCommand('bash -c "pkill -f smorphi_bringup.launch.py"');
}

// 3️⃣ Teleop
function teleopEnable() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 run teleop_twist_keyboard teleop_twist_keyboard"');
}

// 4️⃣ RViz
function rvizStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && rviz2"');
}

// 5️⃣ Auto Transform
function transformEnable() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 run smorphi auto_transform_node --enable"');
}
function transformDisable() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 run smorphi auto_transform_node --disable"');
}

// 6️⃣ MPU ↔ Odometry
function mpuOdom() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 run smorphi mpu_odom_node"');
}

// 7️⃣ Mapping
function mappingStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 launch smorphi mapping.launch.py"');
}
function mappingStop() {
  sendCommand('bash -c "pkill -f mapping.launch.py"');
}

// 8️⃣ Navigation
function navStart() {
  sendCommand('bash -c "source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash && cd ~/smorphi_ws && ros2 launch smorphi nav.launch.py"');
}
function navStop() {
  sendCommand('bash -c "pkill -f nav.launch.py"');
}
