import os
import signal
import subprocess
import time
import json
import re
from pathlib import Path
from typing import List, Literal, Optional

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field


WORKSPACE = Path(os.getenv("AUTOCAR_WORKSPACE", "/workspace"))
SHARED_DIR = Path(os.getenv("AUTOCAR_SHARED_DIR", WORKSPACE / "shared")).resolve()
LOG_DIR = Path(os.getenv("AUTOCAR_LOG_DIR", WORKSPACE / "runtime" / "logs")).resolve()
WORLD_PATH = WORKSPACE / "install" / "autocar_gazebo" / "share" / "autocar_gazebo" / "worlds" / "autocar.world"

ROS_SETUP = (
    "source /opt/ros/foxy/setup.bash && "
    f"source {WORKSPACE}/install/setup.bash && "
    f"cd {WORKSPACE}"
)

app = FastAPI(title="AutoCar ROS 2 Foxy Control API", version="1.0.0")

simulation_process: Optional[subprocess.Popen] = None
last_start_request: Optional["StartSimulationRequest"] = None


class StartSimulationRequest(BaseModel):
    mode: Literal["default", "click", "gazebo", "race", "race_mpc", "race_pure_pursuit"] = Field(
        "default",
        description="default/click: legacy demos, gazebo: Gazebo seul, race*: stacks interactifs de course",
    )
    headless: bool = Field(False, description="Only used by mode=gazebo. false launches Gazebo GUI, true launches gzserver only.")
    control_mode: Literal["manual", "semi", "auto"] = Field("auto", description="Initial mode for race* launches.")
    gui: bool = Field(True, description="Launch Gazebo GUI for race* launches.")
    rviz: bool = Field(True, description="Launch RViz for race* launches.")


class ManualCommandRequest(BaseModel):
    linear_x: float = 0.0
    angular_z: float = 0.0
    duration_sec: float = Field(0.0, ge=0.0, le=60.0)
    rate_hz: float = Field(10.0, gt=0.0, le=50.0)


class ControlModeRequest(BaseModel):
    mode: Literal["manual", "semi", "auto"]


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "odom"


class TextFileRequest(BaseModel):
    path: str
    content: str


def ensure_dirs() -> None:
    SHARED_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)


def checked_shared_path(relative_path: str) -> Path:
    target = (SHARED_DIR / relative_path).resolve()
    if SHARED_DIR not in target.parents and target != SHARED_DIR:
        raise HTTPException(status_code=400, detail="Path must stay inside the shared directory")
    return target


def shell(command: str, timeout: float = 10.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "-lc", f"{ROS_SETUP} && {command}"],
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def parse_control_status_echo(stdout: str) -> Optional[dict]:
    """Extract JSON payload from `ros2 topic echo` output."""
    for line in stdout.splitlines():
        stripped = line.strip()
        if not stripped.startswith("data:"):
            continue
        payload = stripped.split(":", 1)[1].strip()
        if payload.startswith("'") and payload.endswith("'"):
            payload = payload[1:-1]
        elif payload.startswith('"') and payload.endswith('"'):
            payload = payload[1:-1]
        return json.loads(payload)

    match = re.search(r"data:\s*['\"]?(\{.*\})['\"]?", stdout, re.DOTALL)
    if match:
        return json.loads(match.group(1))
    return None


def simulation_status() -> dict:
    if simulation_process is None:
        return {"running": False, "pid": None, "returncode": None}

    returncode = simulation_process.poll()
    return {
        "running": returncode is None,
        "pid": simulation_process.pid,
        "returncode": returncode,
    }


def stop_process(process: Optional[subprocess.Popen], timeout: float = 8.0) -> None:
    if process is None or process.poll() is not None:
        return

    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=3.0)


def launch_command(request: StartSimulationRequest) -> str:
    if request.mode == "default":
        return "ros2 launch launches default_launch.py"

    if request.mode == "click":
        return "ros2 launch launches click_launch.py"

    if request.mode in ("race", "race_mpc", "race_pure_pursuit"):
        launch_file = {
            "race": "race_launch.py",
            "race_mpc": "race_mpc_launch.py",
            "race_pure_pursuit": "race_pure_pursuit_launch.py",
        }[request.mode]
        return (
            f"ros2 launch launches {launch_file} "
            f"control_mode:={request.control_mode} "
            f"gui:={str(request.gui).lower()} "
            f"rviz:={str(request.rviz).lower()}"
        )

    if request.headless:
        return (
            f"gzserver --verbose {WORLD_PATH} "
            "-s libgazebo_ros_init.so "
            "-s libgazebo_ros_factory.so "
            "-s libgazebo_ros_force_system.so"
        )

    return (
        f"gazebo --verbose {WORLD_PATH} "
        "-s libgazebo_ros_init.so "
        "-s libgazebo_ros_factory.so "
        "-s libgazebo_ros_force_system.so"
    )


@app.on_event("startup")
def startup() -> None:
    ensure_dirs()


@app.get("/api/health")
def health() -> dict:
    return {
        "ok": True,
        "ros_distro": os.getenv("ROS_DISTRO", "foxy"),
        "workspace": str(WORKSPACE),
        "simulation": simulation_status(),
    }


@app.get("/api/status")
def status() -> dict:
    topics: List[str] = []
    if simulation_status()["running"]:
        result = shell("ros2 topic list", timeout=5.0)
        if result.returncode == 0:
            topics = [line for line in result.stdout.splitlines() if line]

    return {
        "simulation": simulation_status(),
        "topics": topics,
        "shared_dir": str(SHARED_DIR),
        "logs_dir": str(LOG_DIR),
    }


@app.post("/api/sim/start")
def start_simulation(request: StartSimulationRequest) -> dict:
    global last_start_request, simulation_process

    if simulation_status()["running"]:
        raise HTTPException(status_code=409, detail="Simulation is already running")

    ensure_dirs()
    log_file = LOG_DIR / f"simulation-{int(time.time())}.log"
    command = f"{ROS_SETUP} && exec {launch_command(request)}"

    with log_file.open("ab") as stdout:
        simulation_process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=stdout,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    last_start_request = request
    return {
        "started": True,
        "pid": simulation_process.pid,
        "mode": request.mode,
        "headless": request.headless if request.mode == "gazebo" else None,
        "log_file": str(log_file),
    }


@app.post("/api/sim/stop")
def stop_simulation() -> dict:
    global simulation_process

    stop_process(simulation_process)
    simulation_process = None
    shell("killall -q gzserver gzclient gazebo rviz2 || true", timeout=5.0)
    return {"stopped": True}


@app.post("/api/sim/reset")
def reset_simulation() -> dict:
    request = last_start_request or StartSimulationRequest()
    stop_simulation()
    return start_simulation(request)


@app.post("/api/command/manual")
def manual_command(request: ManualCommandRequest) -> dict:
    twist = (
        "{linear: {x: %.6f, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %.6f}}"
        % (request.linear_x, request.angular_z)
    )

    if request.duration_sec > 0:
        command = (
            "timeout %.3f ros2 topic pub -r %.3f /autocar/manual_cmd_vel geometry_msgs/msg/Twist '%s'"
            % (request.duration_sec, request.rate_hz, twist)
        )
        timeout = request.duration_sec + 3.0
    else:
        command = "ros2 topic pub --once /autocar/manual_cmd_vel geometry_msgs/msg/Twist '%s'" % twist
        timeout = 5.0

    result = shell(command, timeout=timeout)
    if result.returncode not in (0, 124):
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)

    return {"published": True, "topic": "/autocar/manual_cmd_vel"}


@app.post("/api/control/mode")
def set_control_mode(request: ControlModeRequest) -> dict:
    result = shell(
        "ros2 topic pub --once /autocar/control_mode std_msgs/msg/String \"{data: '%s'}\""
        % request.mode,
        timeout=5.0,
    )
    if result.returncode != 0:
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)
    return {"published": True, "topic": "/autocar/control_mode", "mode": request.mode}


@app.post("/api/control/stop")
def stop_vehicle() -> dict:
    result = shell(
        "ros2 topic pub --once /autocar/stop std_msgs/msg/Bool '{data: true}'",
        timeout=5.0,
    )
    if result.returncode != 0:
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)
    return {"stopped": True}


@app.post("/api/control/resume")
def resume_vehicle() -> dict:
    result = shell(
        "ros2 topic pub --once /autocar/resume_auto std_msgs/msg/Bool '{data: true}'",
        timeout=5.0,
    )
    if result.returncode != 0:
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)
    return {"resumed": True, "mode": "auto"}


@app.get("/api/control/status")
def control_status() -> dict:
    result = shell("timeout 3 ros2 topic echo --once /autocar/control_status", timeout=5.0)
    if result.returncode not in (0, 124):
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)
    raw = result.stdout.strip()
    status = None
    try:
        status = parse_control_status_echo(raw)
    except json.JSONDecodeError:
        status = None
    return {"raw": raw, "status": status}


@app.post("/api/navigation/goal")
def navigation_goal(request: GoalRequest) -> dict:
    pose = (
        "{header: {frame_id: '%s'}, pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: %.6f, w: 1.0}}}"
        % (request.frame_id, request.x, request.y, request.yaw)
    )

    result = shell("ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '%s'" % pose, timeout=5.0)
    if result.returncode != 0:
        raise HTTPException(status_code=500, detail=result.stderr or result.stdout)

    return {"published": True, "topic": "/goal_pose"}


@app.get("/api/files")
def list_files() -> dict:
    ensure_dirs()
    files = [
        str(path.relative_to(SHARED_DIR))
        for path in SHARED_DIR.rglob("*")
        if path.is_file()
    ]
    return {"files": sorted(files)}


@app.post("/api/files/upload")
async def upload_file(file: UploadFile = File(...), path: Optional[str] = None) -> dict:
    ensure_dirs()
    target = checked_shared_path(path or file.filename)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(await file.read())
    return {"uploaded": True, "path": str(target.relative_to(SHARED_DIR))}


@app.post("/api/files/text")
def write_text_file(request: TextFileRequest) -> dict:
    ensure_dirs()
    target = checked_shared_path(request.path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(request.content)
    return {"written": True, "path": str(target.relative_to(SHARED_DIR))}


@app.get("/api/files/{path:path}")
def download_file(path: str) -> FileResponse:
    target = checked_shared_path(path)
    if not target.is_file():
        raise HTTPException(status_code=404, detail="File not found")
    return FileResponse(target)


@app.delete("/api/files/{path:path}")
def delete_file(path: str) -> dict:
    target = checked_shared_path(path)
    if not target.exists():
        raise HTTPException(status_code=404, detail="File not found")
    if target.is_dir():
        raise HTTPException(status_code=400, detail="Directory deletion is not supported")
    target.unlink()
    return {"deleted": True}
