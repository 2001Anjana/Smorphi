from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import os
import signal
import json

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
