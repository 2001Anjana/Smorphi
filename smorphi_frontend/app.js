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
  build:     { logId: "log-misc",      dotId: "dot-misc" },
  rviz:      { logId: "log-misc",      dotId: "dot-misc" },
  mpu:       { logId: "log-misc",      dotId: "dot-misc" },
  mapsaver:  { logId: "log-misc",      dotId: "dot-misc" },
};

// ---------------- LOGGING WITH THROTTLING ----------------

const logBuffers = {};
const LOG_UPDATE_INTERVAL = 100;

function initLogBuffer(logId) {
  if (!logBuffers[logId]) {
    logBuffers[logId] = [];
    setInterval(() => flushLogBuffer(logId), LOG_UPDATE_INTERVAL);
  }
}

function flushLogBuffer(logId) {
  const buffer = logBuffers[logId];
  if (!buffer || buffer.length === 0) return;

  const logBox = document.getElementById(logId);
  if (!logBox) return;

  const fragment = document.createDocumentFragment();
  buffer.forEach(({ text, cls }) => {
    const span = document.createElement("span");
    if (cls) span.className = cls;
    span.textContent = text + "\n";
    fragment.appendChild(span);
  });
  logBox.appendChild(fragment);
  buffer.length = 0;

  if (logBox.scrollHeight - logBox.scrollTop <= logBox.clientHeight + 50) {
    logBox.scrollTop = logBox.scrollHeight;
  }
}

function getLogStyle(text) {
  const t = text.toLowerCase();
  if (t.includes("error") || t.includes("failed") || t.includes("fatal")) return "log-error";
  if (t.includes("warn"))           return "log-warn";
  if (t.includes("command_finished")) return "log-ok";
  if (t.includes("running:") || t.includes("session ready") || t.includes("websocket")) return "log-info";
  if (t.includes("kill result") || t.startsWith("[system]")) return "log-system";
  return "";
}

function log(sessionName, rawMessage) {
  const cfg = SESSION_CONFIG[sessionName] || { logId: "log-misc", dotId: "dot-misc" };
  const logId = cfg.logId;
  initLogBuffer(logId);
  const cleaned = rawMessage.replace(/^\[[\w-]+\]\s*/, "");
  const cls = getLogStyle(rawMessage);
  logBuffers[logId].push({ text: cleaned, cls });

  const dot = document.getElementById(cfg.dotId);
  if (dot) {
    if (rawMessage.includes("Process started"))        dot.className = "term-session-dot active";
    else if (rawMessage.includes("COMMAND_FINISHED"))  dot.className = "term-session-dot finished";
    else if (rawMessage.includes("error") || rawMessage.toLowerCase().includes("failed"))
                                                       dot.className = "term-session-dot error";
  }
}

function sysLog(msg) {
  log("build", `[system] ${msg}`);
}

function clearLog(panelName) {
  const cfg = SESSION_CONFIG[panelName] || { logId: `log-${panelName}` };
  const logId = cfg.logId || `log-${panelName}`;
  const el = document.getElementById(logId);
  if (el) el.innerHTML = "";
  if (logBuffers[logId]) logBuffers[logId].length = 0;
}

// ---------------- CONNECTION ----------------

function connectRos() {
  sysLog("Connecting to " + PI_IP + "...");
  fetch(`http://${PI_IP}:8000/health`)
    .then(res => res.json())
    .then(() => {
      document.getElementById("connectionStatus").textContent = "CONNECTED";
      document.getElementById("statusDot").className = "status-dot connected";
      sysLog("Pi is reachable.");
    })
    .catch(() => {
      document.getElementById("connectionStatus").textContent = "FAILED";
      document.getElementById("statusDot").className = "status-dot failed";
      sysLog("ERROR: Could not reach Pi");
    });
}

// ---------------- SESSION MANAGEMENT ----------------

function getSession(sessionName) {
  const existing = sessions[sessionName];
  if (existing && existing.readyState === WebSocket.OPEN) return existing;

  const ws = new WebSocket(`ws://${PI_IP}:8000/ws/logs/${sessionName}`);
  ws.onmessage = (event) => log(sessionName, event.data);
  ws.onopen    = () => sysLog(`[${sessionName}] connected`);
  ws.onerror   = () => sysLog(`[${sessionName}] WebSocket error`);
  ws.onclose   = () => {
    const cfg = SESSION_CONFIG[sessionName];
    if (cfg) {
      const dot = document.getElementById(cfg.dotId);
      if (dot && dot.classList.contains("active")) dot.className = "term-session-dot finished";
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
  sysLog(`Stopping ${sessionName}...`);
  fetch(`http://${PI_IP}:8000/kill/${sessionName}`)
    .then(res => res.json())
    .then(data => {
      log(sessionName, `Kill result: ${data.status}`);
      const cfg = SESSION_CONFIG[sessionName];
      if (cfg) {
        const dot = document.getElementById(cfg.dotId);
        if (dot) dot.className = "term-session-dot finished";
      }
    })
    .catch(err => sysLog(`Kill failed: ${err}`));
}

// ---------------- ROS SERVICE CALLS ----------------

async function callRosService(serviceName, data, sessionName = "transform") {
  try {
    const response = await fetch(`http://${PI_IP}:8000/ros/service/${serviceName}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'std_srvs/srv/SetBool', data })
    });
    const result = await response.json();
    log(sessionName, `Service call /${serviceName}: ${result.status}`);
    if (result.stdout) log(sessionName, result.stdout);
    if (result.stderr) log(sessionName, result.stderr);
    return result;
  } catch (err) {
    log(sessionName, `Service call failed: ${err}`);
  }
}

// ---------------- BUTTON COMMANDS ----------------

const ROS_SOURCE = 'source /opt/ros/humble/setup.bash && source ~/smorphi_ws/install/setup.bash';
const UNBUF = 'stdbuf -oL -eL';

function build() {
  sendToSession("build", `bash -c "${ROS_SOURCE} && cd ~/smorphi_ws && colcon build --event-handlers console_direct+ --symlink-install"`);
}

function bringupStart() {
  sendToSession("bringup", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py"`);
}
function bringupStop() { killSession("bringup"); }

// ─── TELEOP ──────────────────────────────────────────────────────────────────

async function teleopEnable() {
  log("teleop", "Starting teleop publisher...");
  try {
    const response = await fetch(`http://${PI_IP}:8000/teleop/start`, { method: 'POST' });
    const result = await response.json();

    if (result.status === 'started' || result.status === 'already_running') {
      log("teleop", `✓ Teleop publisher running (PID ${result.pid || '?'})`);
      log("teleop", "Press keys or click buttons to move. Release key = robot stops.");

      document.getElementById("teleopModal").style.display = "flex";
      document.getElementById("teleopModal").focus();

      const dot = document.getElementById("dot-teleop");
      if (dot) dot.className = "term-session-dot active";
    } else {
      log("teleop", "Failed to start teleop publisher");
    }
  } catch (err) {
    log("teleop", `Error starting teleop: ${err}`);
  }
}

async function teleopDisable() {
  document.getElementById("teleopModal").style.display = "none";
  log("teleop", "Stopping teleop publisher...");
  try {
    const response = await fetch(`http://${PI_IP}:8000/teleop/stop`, { method: 'POST' });
    const result = await response.json();
    if (result.status === 'stopped') log("teleop", "✓ Teleop disabled, robot stopped");
  } catch (err) {
    log("teleop", `Error stopping teleop: ${err}`);
  }
  const dot = document.getElementById("dot-teleop");
  if (dot) dot.className = "term-session-dot finished";
}

// ─── Send velocity update to the persistent publisher ────────────────────────
//
// The backend now runs ONE long-lived `ros2 topic pub --rate 10` process.
// Calling /teleop/cmd restarts it with the new velocity.
// We debounce so rapid key-repeat events don't spam restarts.

let _teleopCmdTimer = null;
let _pendingVel = null;

async function sendTeleopCommand(linear_x, linear_y, angular_z) {
  _pendingVel = { linear_x, linear_y, angular_z };

  // Debounce: wait 40 ms before actually firing, so key-repeat
  // events for the same key collapse into one request.
  if (_teleopCmdTimer) return;

  _teleopCmdTimer = setTimeout(async () => {
    _teleopCmdTimer = null;
    const vel = _pendingVel;
    if (!vel) return;
    try {
      await fetch(`http://${PI_IP}:8000/teleop/cmd`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(vel)
      });
    } catch (err) {
      // Suppress spam when modal is closed
      const modal = document.getElementById("teleopModal");
      if (modal && modal.style.display === 'flex') {
        log("teleop", `Cmd failed: ${err}`);
      }
    }
  }, 40);
}

// ─── Keyboard handling ────────────────────────────────────────────────────────

const TELEOP_KEY_MAP = {
  'i': { linear_x:  0.5, linear_y:  0.0, angular_z:  0.0 },  // forward
  'u': { linear_x:  0.5, linear_y:  0.0, angular_z:  1.0 },  // fwd-left
  'o': { linear_x:  0.5, linear_y:  0.0, angular_z: -1.0 },  // fwd-right
  'j': { linear_x:  0.0, linear_y:  0.0, angular_z:  1.0 },  // rotate left
  'k': { linear_x:  0.0, linear_y:  0.0, angular_z:  0.0 },  // stop
  'l': { linear_x:  0.0, linear_y:  0.0, angular_z: -1.0 },  // rotate right
  'm': { linear_x: -0.5, linear_y:  0.0, angular_z:  1.0 },  // bwd-left
  ',': { linear_x: -0.5, linear_y:  0.0, angular_z:  0.0 },  // backward
  '.': { linear_x: -0.5, linear_y:  0.0, angular_z: -1.0 },  // bwd-right
  // Holonomic / strafe
  'I': { linear_x:  0.5, linear_y:  0.0, angular_z:  0.0 },
  'U': { linear_x:  0.5, linear_y:  0.5, angular_z:  0.0 },
  'O': { linear_x:  0.5, linear_y: -0.5, angular_z:  0.0 },
  'J': { linear_x:  0.0, linear_y:  0.5, angular_z:  0.0 },
  'K': { linear_x:  0.0, linear_y:  0.0, angular_z:  0.0 },
  'L': { linear_x:  0.0, linear_y: -0.5, angular_z:  0.0 },
  'M': { linear_x: -0.5, linear_y: -0.5, angular_z:  0.0 },
  '<': { linear_x: -0.5, linear_y: -0.5, angular_z:  0.0 },
  '>': { linear_x: -0.5, linear_y:  0.5, angular_z:  0.0 },
  // Space = emergency stop
  ' ': { linear_x:  0.0, linear_y:  0.0, angular_z:  0.0 },
};

// Track which keys are currently held so keyup can send stop
const _keysHeld = new Set();

document.addEventListener('keydown', (e) => {
  const modal = document.getElementById('teleopModal');
  if (!modal || modal.style.display !== 'flex') return;

  const key = e.key;
  const vel = TELEOP_KEY_MAP[key];
  if (!vel) return;

  e.preventDefault();

  // Ignore auto-repeat — we already sent this on the real keydown
  if (_keysHeld.has(key)) return;
  _keysHeld.add(key);

  sendTeleopCommand(vel.linear_x, vel.linear_y, vel.angular_z);
  highlightKey(key);
  log("teleop", `▶ ${key}  lin_x=${vel.linear_x}  ang_z=${vel.angular_z}`);
});

document.addEventListener('keyup', (e) => {
  const modal = document.getElementById('teleopModal');
  if (!modal || modal.style.display !== 'flex') return;

  const key = e.key;
  if (!TELEOP_KEY_MAP[key]) return;

  e.preventDefault();
  _keysHeld.delete(key);

  // Only send stop when ALL movement keys are released
  if (_keysHeld.size === 0) {
    sendTeleopCommand(0.0, 0.0, 0.0);
    log("teleop", "■ key released → stop");
  }
});

// Virtual on-screen buttons (mousedown/up for press-and-hold behaviour)
function sendTeleopKey(key) {
  const vel = TELEOP_KEY_MAP[key];
  if (vel) {
    sendTeleopCommand(vel.linear_x, vel.linear_y, vel.angular_z);
    highlightKey(key);
    log("teleop", `Btn: ${key} → lin_x=${vel.linear_x}  ang_z=${vel.angular_z}`);
  }
}

function stopTeleopKey() {
  sendTeleopCommand(0.0, 0.0, 0.0);
  log("teleop", "■ btn released → stop");
}

function highlightKey(key) {
  const btn = document.querySelector(`[data-key="${key}"]`);
  if (btn) {
    btn.classList.add('key-pressed');
    setTimeout(() => btn.classList.remove('key-pressed'), 200);
  }
}

// ─── Other commands ───────────────────────────────────────────────────────────

function rvizStart() {
  sysLog("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  sysLog("⚠️  RViz2 must run on YOUR LAPTOP, not the Pi!");
  sysLog("Open a terminal on your laptop and run:");
  sysLog("export ROS_DOMAIN_ID=0");
  sysLog("source /opt/ros/humble/setup.bash");
  sysLog("rviz2");
  sysLog("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

function transformEnable()  { callRosService("shape_auto/enable", { data: true  }, "transform"); }
function transformDisable() { callRosService("shape_auto/enable", { data: false }, "transform"); }

function mpuOdom() {
  sendToSession("mpu", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 run smorphi mpu_odom_node"`);
}

function mappingStart() {
  sendToSession("mapping", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_mapper_online_async_launch.py"`);
}
function mappingStop() { killSession("mapping"); }

function navStart() {
  sendToSession("nav", `bash -c "${ROS_SOURCE} && ${UNBUF} ros2 launch smorphi_ros2_launchers smorphi_nav2.launch.py"`);
}
function navStop() { killSession("nav"); }

function saveMap(mapName = "my_map") {
  sendToSession("mapsaver", `bash -c "${ROS_SOURCE} && ros2 run nav2_map_server map_saver_cli -f ${mapName}"`);
}
