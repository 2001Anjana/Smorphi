from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import os
import signal
import json
import httpx
import yaml
from dotenv import load_dotenv

load_dotenv()  # reads backend/.env  →  GEMINI_API_KEY

# ─── LLM / Navigation config ─────────────────────────────────────────────────

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_URL = (
    "https://generativelanguage.googleapis.com/v1/models/"
    "gemini-2.5-flash:generateContent"
)

# Path to the waypoint YAML saved by waypoint_manager during mapping
WAYPOINT_FILE = os.path.expanduser("~/smorphi_waypoints.yaml")


def load_waypoint_labels() -> list[str]:
    """Read ~/smorphi_waypoints.yaml and return the list of label strings."""
    if not os.path.exists(WAYPOINT_FILE):
        return []
    try:
        with open(WAYPOINT_FILE, "r") as f:
            data = yaml.safe_load(f)
        if data and isinstance(data, dict):
            return list(data.keys())
    except Exception:
        pass
    return []


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Registry of named running processes (for bringup, mapping, nav etc.)
running_processes: dict[str, asyncio.subprocess.Process] = {}

ROS_SOURCE = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/smorphi_ws/install/setup.bash"
)

# Path to the persistent teleop node script (same folder as this file)
TELEOP_NODE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop_vel_node.py")


@app.get("/health")
def health():
    return {"status": "ok"}


# ─── WAYPOINT NAV PROCESS MANAGEMENT ──────────────────────────────────────────
#
# go_to_waypoint is an INTERACTIVE process that reads waypoint labels from
# stdin.  We keep it running as a long-lived subprocess and pipe labels to it.
# ─────────────────────────────────────────────────────────────────────────────

_waypoint_proc: asyncio.subprocess.Process | None = None
_waypoint_lock = asyncio.Lock()
_waypoint_ws_clients: list[WebSocket] = []  # clients streaming waypoint logs


async def _broadcast_waypoint_log(msg: str):
    """Send a log line to all connected waypoint WebSocket clients."""
    dead = []
    for ws in _waypoint_ws_clients:
        try:
            await ws.send_text(f"[waypoint] {msg}")
        except Exception:
            dead.append(ws)
    for ws in dead:
        _waypoint_ws_clients.remove(ws)


async def _waypoint_stdout_reader():
    """Background task: reads stdout from go_to_waypoint and broadcasts it."""
    global _waypoint_proc
    if _waypoint_proc is None:
        return
    try:
        while _waypoint_proc and _waypoint_proc.returncode is None:
            line = await _waypoint_proc.stdout.readline()
            if not line:
                break
            text = line.decode().rstrip()
            await _broadcast_waypoint_log(text)
    except Exception:
        pass
    await _broadcast_waypoint_log("COMMAND_FINISHED")


@app.post("/waypoint/launch")
async def waypoint_launch():
    """Launch `ros2 run smorphi_ros2_launchers go_to_waypoint` with stdin attached."""
    global _waypoint_proc

    async with _waypoint_lock:
        if _waypoint_proc is not None and _waypoint_proc.returncode is None:
            return {"status": "already_running", "pid": _waypoint_proc.pid}

        cmd = (
            f'bash -c "{ROS_SOURCE} && '
            f'stdbuf -oL -eL ros2 run smorphi_ros2_launchers go_to_waypoint"'
        )
        _waypoint_proc = await asyncio.create_subprocess_shell(
            cmd,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True,
        )

    # Start background reader to broadcast stdout
    asyncio.create_task(_waypoint_stdout_reader())

    await _broadcast_waypoint_log(f"Process started (PID {_waypoint_proc.pid})")
    return {"status": "started", "pid": _waypoint_proc.pid}


@app.post("/waypoint/stop")
async def waypoint_stop():
    """Kill the running go_to_waypoint process."""
    global _waypoint_proc

    async with _waypoint_lock:
        if _waypoint_proc is not None:
            try:
                os.killpg(os.getpgid(_waypoint_proc.pid), signal.SIGINT)
                await asyncio.sleep(0.5)
                if _waypoint_proc.returncode is None:
                    os.killpg(os.getpgid(_waypoint_proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
            _waypoint_proc = None

    await _broadcast_waypoint_log("Process stopped")
    return {"status": "stopped"}


@app.get("/waypoint/status")
async def waypoint_status():
    """Check whether go_to_waypoint is currently running."""
    if _waypoint_proc is not None and _waypoint_proc.returncode is None:
        return {"running": True, "pid": _waypoint_proc.pid}
    return {"running": False}


@app.websocket("/ws/waypoint")
async def waypoint_ws(websocket: WebSocket):
    """Stream go_to_waypoint stdout logs to the frontend in real time."""
    await websocket.accept()
    _waypoint_ws_clients.append(websocket)
    await websocket.send_text("[waypoint] Connected to waypoint log stream")
    try:
        while True:
            await websocket.receive_text()  # keep-alive; ignore input
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in _waypoint_ws_clients:
            _waypoint_ws_clients.remove(websocket)


# ─── VOICE NAVIGATION ─────────────────────────────────────────────────────────

async def _extract_location_with_gemini(transcript: str, known_labels: list[str]) -> str:
    """
    Ask Gemini to extract the destination location name from a free-form voice
    transcript.  Returns the matched location label or 'unknown'.
    """
    location_list = ", ".join(known_labels)
    prompt = (
        f"You are a navigation assistant for a mobile robot. "
        f"The available locations are: {location_list}. "
        f"The user will give a voice command. Extract ONLY the destination location name "
        f"from the command and reply with that location name exactly as listed above. "
        f"Do NOT add any explanation, punctuation, or extra words. "
        f"Examples:\n"
        f"  Voice: \"go to Home\" → Home\n"
        f"  Voice: \"navigate to cutter machine\" → cutter machine\n"
        f"  Voice: \"take me to wrapper machine\" → wrapper machine\n"
        f"  Voice: \"Home\" → Home\n"
        f"  Voice: \"hello\" → unknown\n"
        f"If no location matches, reply with exactly: unknown\n"
        f"Voice command: \"{transcript}\""
    )

    payload = {
        "contents": [{"parts": [{"text": prompt}]}],
        "generationConfig": {"maxOutputTokens": 50, "temperature": 0.1},
    }

    headers = {"Content-Type": "application/json"}
    params  = {"key": GEMINI_API_KEY}

    async with httpx.AsyncClient(timeout=15.0) as client:
        resp = await client.post(GEMINI_URL, json=payload, headers=headers, params=params)
        resp.raise_for_status()
        data = resp.json()

    # gemini-2.5-flash is a thinking model: parts[0] may be reasoning,
    # the actual answer is in the LAST text part.
    parts = (
        data.get("candidates", [{}])[0]
            .get("content", {})
            .get("parts", [])
    )
    raw = ""
    for part in reversed(parts):
        text = part.get("text", "").strip()
        if text:
            raw = text
            break
    return raw


@app.post("/voice/navigate")
async def voice_navigate(request_data: dict):
    """
    Accepts: { "transcript": "go to cutter machine" }
    1. Checks go_to_waypoint process is running.
    2. Calls Gemini to extract the location label.
    3. Validates against known waypoint labels from YAML.
    4. Pipes the label to go_to_waypoint stdin.
    """
    global _waypoint_proc

    transcript = request_data.get("transcript", "").strip()
    if not transcript:
        return {"status": "error", "message": "Empty transcript"}

    if not GEMINI_API_KEY:
        return {"status": "error", "message": "GEMINI_API_KEY not set in .env"}

    # Check waypoint process is running
    if _waypoint_proc is None or _waypoint_proc.returncode is not None:
        return {
            "status": "not_running",
            "message": "Waypoint navigation is not running. Launch it first.",
        }

    # Load current labels from YAML
    known_labels = load_waypoint_labels()
    if not known_labels:
        return {"status": "error", "message": "No waypoints found in YAML file"}

    # Step 1 – extract location via LLM
    try:
        extracted = await _extract_location_with_gemini(transcript, known_labels)
    except Exception as e:
        return {"status": "error", "message": f"LLM call failed: {e}"}

    # Step 2 – validate against known labels (case-insensitive match)
    matched_label = None
    for label in known_labels:
        if label.lower() == extracted.lower():
            matched_label = label
            break

    if matched_label is None:
        return {
            "status": "unknown_location",
            "extracted": extracted,
            "transcript": transcript,
            "available": known_labels,
        }

    # Step 3 – pipe the label to go_to_waypoint stdin
    try:
        _waypoint_proc.stdin.write(f"{matched_label}\n".encode())
        await _waypoint_proc.stdin.drain()
    except Exception as e:
        return {"status": "error", "message": f"Failed to send label to process: {e}"}

    await _broadcast_waypoint_log(f"Voice → navigating to \"{matched_label}\"")

    return {
        "status": "dispatched",
        "location": matched_label,
        "transcript": transcript,
    }


@app.get("/voice/locations")
def voice_locations():
    """Return the list of known waypoint labels from the YAML file."""
    labels = load_waypoint_labels()
    return {"locations": labels}


# ─── TELEOP ───────────────────────────────────────────────────────────────────
#
# We launch teleop_vel_node.py as a SINGLE persistent process.
# It is a real ROS2 node that publishes /cmd_vel at 10 Hz.
# To change velocity we simply write "lx,ly,az\n" to its stdin.
# No process restarts, no spawning ros2 CLI on every keypress.
# ─────────────────────────────────────────────────────────────────────────────

_teleop_proc: asyncio.subprocess.Process | None = None
_teleop_lock = asyncio.Lock()


@app.post("/teleop/start")
async def teleop_start():
    global _teleop_proc

    async with _teleop_lock:
        if _teleop_proc is not None and _teleop_proc.returncode is None:
            return {"status": "already_running"}

        cmd = f'bash -c "{ROS_SOURCE} && python3 {TELEOP_NODE_PATH}"'
        _teleop_proc = await asyncio.create_subprocess_shell(
            cmd,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
            start_new_session=True,
        )

    return {"status": "started", "pid": _teleop_proc.pid}


@app.post("/teleop/stop")
async def teleop_stop():
    global _teleop_proc

    async with _teleop_lock:
        if _teleop_proc is not None:
            # Send zero velocity before killing
            try:
                _teleop_proc.stdin.write(b"0.0,0.0,0.0\n")
                await _teleop_proc.stdin.drain()
                await asyncio.sleep(0.15)
            except Exception:
                pass

            try:
                os.killpg(os.getpgid(_teleop_proc.pid), signal.SIGINT)
                await asyncio.sleep(0.3)
                if _teleop_proc.returncode is None:
                    os.killpg(os.getpgid(_teleop_proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

            _teleop_proc = None

    return {"status": "stopped"}


@app.post("/teleop/cmd")
async def teleop_command(request_data: dict):
    """
    Write the new velocity directly to the node's stdin.
    The node publishes it on its next 10Hz tick (<= 100ms).
    No process restart — just a single stdin write().
    """
    global _teleop_proc

    if _teleop_proc is None or _teleop_proc.returncode is not None:
        return {"status": "not_running"}

    lx = float(request_data.get("linear_x", 0.0))
    ly = float(request_data.get("linear_y", 0.0))
    az = float(request_data.get("angular_z", 0.0))

    try:
        _teleop_proc.stdin.write(f"{lx},{ly},{az}\n".encode())
        await _teleop_proc.stdin.drain()
    except Exception as e:
        return {"status": "error", "message": str(e)}

    return {"status": "ok", "linear_x": lx, "linear_y": ly, "angular_z": az}


# ─── ROS service calls ────────────────────────────────────────────────────────

@app.post("/ros/service/{service_name}")
async def call_ros_service(service_name: str, request_data: dict):
    try:
        srv_type = request_data.get("type", "std_srvs/srv/SetBool")
        data = json.dumps(request_data.get("data", {}))
        cmd = (
            f'bash -c "{ROS_SOURCE} && '
            f'ros2 service call /{service_name} {srv_type} \\"{data}\\""'
        )
        process = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await process.communicate()
        return {
            "status": "success" if process.returncode == 0 else "error",
            "returncode": process.returncode,
            "stdout": stdout.decode(),
            "stderr": stderr.decode(),
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ─── Kill a named session ─────────────────────────────────────────────────────

@app.get("/kill/{session_name}")
async def kill_session(session_name: str):
    process = running_processes.get(session_name)
    if process is None:
        return {"status": "not_found", "session": session_name}
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            await asyncio.wait_for(process.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            await asyncio.wait_for(process.wait(), timeout=1.0)
    except ProcessLookupError:
        pass
    except Exception as e:
        return {"status": "error", "message": str(e)}
    finally:
        running_processes.pop(session_name, None)
    return {"status": "killed", "session": session_name}


# ─── WebSocket log streaming ──────────────────────────────────────────────────

async def stream_command(cmd: str, session_name: str, websocket: WebSocket):
    old = running_processes.get(session_name)
    if old is not None:
        try:
            os.killpg(os.getpgid(old.pid), signal.SIGINT)
            await asyncio.sleep(0.5)
            os.killpg(os.getpgid(old.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
        running_processes.pop(session_name, None)

    process = None
    try:
        process = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            stdin=asyncio.subprocess.PIPE,
            start_new_session=True,
        )
        running_processes[session_name] = process
        await websocket.send_text(f"[{session_name}] Process started (PID {process.pid})")

        while True:
            line = await process.stdout.readline()
            if not line:
                break
            try:
                await websocket.send_text(f"[{session_name}] {line.decode().rstrip()}")
            except Exception:
                break

        await process.wait()
    except Exception as e:
        await websocket.send_text(f"[{session_name}] ERROR: {str(e)}")
    finally:
        running_processes.pop(session_name, None)
        if process is not None:
            try:
                await websocket.send_text(
                    f"[{session_name}] COMMAND_FINISHED (exit {process.returncode})"
                )
            except Exception:
                pass


@app.websocket("/ws/logs/{session_name}")
async def websocket_logs(websocket: WebSocket, session_name: str):
    await websocket.accept()
    await websocket.send_text(f"[{session_name}] Session ready")
    try:
        while True:
            data = await websocket.receive_text()
            await websocket.send_text(f"[{session_name}] Running: {data}")
            await stream_command(data, session_name, websocket)
    except WebSocketDisconnect:
        process = running_processes.get(session_name)
        if process:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                await asyncio.sleep(0.5)
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
            running_processes.pop(session_name, None)
