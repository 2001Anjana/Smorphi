const PI_IP = "10.0.0.6";

// One WebSocket per named session
const sessions = {};

// Map each session to its log panel id and dot id
const SESSION_CONFIG = {
  bringup:   { logId: "log-bringup",   dotId: "dot-bringup" },
  teleop:    { logId: "log-teleop",    dotId: "dot-teleop" },
  mapping:   { logId: "log-mapping",   dotId: "dot-mapping" },
  nav:       { logId: "log-nav",       dotId: "dot-nav" },
  transform: { logId: "log-transform", dotId: "dot-transform" },
  // build, rviz, mpu, mapsaver all share the "misc" panel
  build:     { logId: "log-misc",      dotId: "dot-misc" },
  rviz:      { logId: "log-misc",      dotId: "dot-misc" },
  mpu:       { logId: "log-misc",      dotId: "dot-misc" },
  mapsaver:  { logId: "log-misc",      dotId: "dot-misc" },
};

// ---------------- LOGGING ----------------

function getLogStyle(text) {
  const t = text.toLowerCase();
  if (t.includes("error") || t.includes("failed") || t.includes("fatal"))
    return "log-error";
  if (t.includes("warn"))
    return "log-warn";
  if (t.includes("command_finished"))
    return "log-ok";
  if (t.includes("running:") || t.includes("session ready") || t.includes("websocket"))
    return "log-info";
  if (t.includes("kill result") || t.startsWith("[system]"))
    return "log-system";
  return "";
}

function log(sessionName, rawMessage) {
  const cfg = SESSION_CONFIG[sessionName] || { logId: "log-misc", dotId: "dot-misc" };
  const logBox = document.getElementById(cfg.logId);
  if (!logBox) return;

  const cls = getLogStyle(rawMessage);
  const span = document.createElement("span");
  if (cls) span.className = cls;

  // Strip the [sessionName] prefix the backend adds, since each panel is already labeled
  const cleaned = rawMessage.replace(/^\[[\w-]+\]\s*/, "");
  span.textContent = cleaned + "\n";

  logBox.appendChild(span);
  logBox.scrollTop = logBox.scrollHeight;

  // Update status dot based on message
  const dot = document.getElementById(cfg.dotId);
  if (dot) {
    if (rawMessage.includes("Process started")) {
      dot.className = "term-session-dot active";
    } else if (rawMessage.includes("COMMAND_FINISHED")) {
      dot.className = "term-session-dot finished";
    } else if (rawMessage.includes("error") || rawMessage.toLowerCase().includes("failed")) {
      dot.className = "term-session-dot error";
    }
  }
}

// System-level messages go to the misc panel
function sysLog(msg) {
  log("build", `[system] ${msg}`);
}

function clearLog(panelName) {
  // Find all sessions using this panel and clear
  const cfg = SESSION_CONFIG[panelName] || { logId: `log-${panelName}` };
  const logId = cfg.logId || `log-${panelName}`;
  const el = document.getElementById(logId);
  if (el) el.innerHTML = "";
}

// ---------------- CONNECTION ----------------

function connectRos() {
  sysLog("Attempting connection to " + PI_IP + "...");
  fetch(`http://${PI_IP}:8000/health`)
    .then(res => res.json())
    .then(() => {
      const status = document.getElementById("connectionStatus");
      const dot = document.getElementById("statusDot");
      status.textContent = "CONNECTED";
      dot.className = "status-dot connected";
      sysLog("Health check passed. Pi is reachable.");
    })
    .catch(() => {
      const status = document.getElementById("connectionStatus");
      const dot = document.getElementById("statusDot");
      status.textContent = "FAILED";
      dot.className = "status-dot failed";
      sysLog("ERROR: Could not reach Pi at " + PI_IP);
    });
}

// ---------------- SESSION MANAGEMENT ----------------

function getSession(sessionName) {
  const existing = sessions[sessionName];
  if (existing && existing.readyState === WebSocket.OPEN) {
    return existing;
  }

  const ws = new WebSocket(`ws://${PI_IP}:8000/ws/logs/${sessionName}`);

  ws.onmessage = (event) => log(sessionName, event.data);

  ws.onopen = () => {
    sysLog(`[${sessionName}] connection established`);
  };

  ws.onerror = () => {
    sysLog(`[${sessionName}] WebSocket error — is the backend running?`);
  };

  ws.onclose = () => {
    const dot = document.getElementById(SESSION_CONFIG[sessionName]?.dotId);
    if (dot && dot.classList.contains("active")) {
      dot.className = "term-session-dot finished";
    }
    delete sessions[sessionName];
  };

  sessions[sessionName] = ws;
  return ws;
}

function sendToSession(sessionName, cmd) {
  const ws = getSession(sessionName);
  if (ws.readyState === WebSocket.OPEN) {
    ws.send(cmd);
  } else {
    ws.addEventListener("open", () => ws.send(cmd), { once: true });
  }
}

function killSession(sessionName) {
  sysLog(`Sending stop signal → ${sessionName}`);
  fetch(`http://${PI_IP}:8000/kill/${sessionName}`)
    .then(res => res.json())
    .then(data => {
      log(sessionName, `[${sessionName}] Kill result: ${data.status}`);
      const dot = document.getElementById(SESSION_CONFIG[sessionName]?.dotId);
      if (dot) dot.className = "term-session-dot finished";
    })
    .catch(err => sysLog(`Kill request failed for ${sessionName}: ${err}`));
}

// ---------------- BUTTON COMMANDS ----------------

// stdbuf -oL forces line-buffered output for programs that detect non-TTY and buffer
// This fixes teleop_twist_keyboard output not appearing until the process ends
const ROS_SOURCE = 'source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash';
const UNBUF = 'stdbuf -oL -eL';  // force line-buffered stdout + stderr

// 1️⃣ Build
function build() {
  sendToSession("build", `bash -c "${ROS_SOURCE} && cd ~/smorphi_ws && colcon build"`);
}

// 2️⃣ Bringup
function bringupStart() {
  sendToSession("bringup", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py"`);
}
function bringupStop() {
  killSession("bringup");
}

// 3️⃣ Teleop
// stdbuf is critical here: teleop_twist_keyboard detects it's not in a TTY and
// fully buffers stdout, causing output to appear only when the process exits.
function teleopEnable() {
  sendToSession("teleop", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 run teleop_twist_keyboard teleop_twist_keyboard"`);
}

// 4️⃣ RViz
function rvizStart() {
  sendToSession("rviz", `bash -c "${ROS_SOURCE} && ${UNBUF} rviz2"`);
}

// 5️⃣ Transform
function transformEnable() {
  sendToSession("transform", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 run smorphi auto_transform_node --enable"`);
}
function transformDisable() {
  killSession("transform");
}

// 6️⃣ MPU Odom
function mpuOdom() {
  sendToSession("mpu", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 run smorphi mpu_odom_node"`);
}

// 7️⃣ Mapping
function mappingStart() {
  sendToSession("mapping", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_mapper_online_async_launch.py"`);
}
function mappingStop() {
  killSession("mapping");
}

// 8️⃣ Navigation
function navStart() {
  sendToSession("nav", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_nav2.launch.py"`);
}
function navStop() {
  killSession("nav");
}

// 9️⃣ Map Saver
function saveMap(mapName = "my_map") {
  sendToSession("mapsaver", `bash -c "${ROS_SOURCE} && ros2 run nav2_map_server map_saver_cli -f ${mapName}"`);
}
