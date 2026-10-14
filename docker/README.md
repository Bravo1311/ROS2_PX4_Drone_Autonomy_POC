# Docker usage

The whole stack (ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic, Micro XRCE-DDS Agent, `px4_msgs`, `px4_ros_com`, this repo, PyTorch with CUDA) is packaged in one image, `px4-ros2-autonomy:latest`. `px4_msgs`, `px4_ros_com` and PX4 are fetched at pinned commits during the build, so only this repo is needed on the host.

All commands below are run from the repository root (the folder that contains `docker-compose.yaml`).

> Status: the image builds, torch sees the GPU, and PX4 SITL starts and connects to the XRCE agent (verified headless). The Gazebo GUI path is verified headless only.

## Prerequisites (host, once)

1. Docker Engine with Compose v2.
2. NVIDIA driver + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then register it with Docker:
   ```bash
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   docker run --rm --gpus all ubuntu:24.04 nvidia-smi   # should print your GPU
   ```
3. X11 access for Gazebo/RViz windows (`run.sh` does this for you): `xhost +local:docker`
4. Joystick group: the image has no group named `input`, so compose adds the host's numeric GID. Check yours with `getent group input` (default in the compose file is `107`).

## Configuration (`.env`)

Create a `.env` file next to `docker-compose.yaml`. It is git-ignored; never commit it.

```bash
GEMINI_API_KEY=...
OPENROUTER_API_KEY=...
ROS_DOMAIN_ID=0
INPUT_GID=107          # output of: getent group input
```

## Build

```bash
docker compose build
```

Build args (defaults are in `docker-compose.yaml` and the `Dockerfile`; override via `.env` or `--build-arg`):

| Arg | Meaning |
|-----|---------|
| `PX4_REF` | PX4-Autopilot commit (full 40-char SHA, not abbreviated) |
| `PX4_MSGS_REF` | `px4_msgs` commit. **Must match `PX4_REF`** (message definitions have to match the firmware) |
| `PX4_ROS_COM_REF` | `px4_ros_com` commit |
| `BUILD_JOBS` | parallel compile jobs (default 8; lower it if the build gets OOM-killed) |
| `XRCE_REF` | Micro XRCE-DDS Agent tag (`Dockerfile` default `v3.0.1`; not passed through compose, edit the `ARG` or use `docker build --build-arg`) |

The first build is long (PX4 compile, CUDA torch download, `colcon build`). Later builds reuse cached layers; the image is about 14 GB.

## Daily workflow

```bash
./run.sh                                              # allows X11, starts the container
docker exec -it px4-ros2-autonomy /entrypoint.sh bash # shell inside the container (repeat per terminal)

# inside the container
ros2 launch px4_bringup minimal.launch.py             # PX4 SITL + Gazebo + XRCE agent + RViz
# second shell
ros2 launch px4_bringup slam.launch.py mode:=mapping  # perception bridges (incl. /clock) + SLAM
```

The remaining terminals from [Launch Sequence](../README.md#launch-sequence) work the same way, each in its own `docker exec` shell.

- `minimal.launch.py` already starts PX4 and the XRCE agent. **Do not also run `docker/start_sim.sh`**: both use UDP port 8888. `start_sim.sh` is only a standalone alternative.
- The old absolute paths (`/home/bravo1311/px4_ros2_ws/...`) resolve inside the container through a symlink to `/ws`. See [Paths](#paths-avoid-hard-coding-them).
- When finished: `Ctrl+C` the launches, then `docker compose down`.

### Shell shortcuts

Interactive shells in the container load ROS and these shortcuts from `docker/ros_aliases.sh`
(sourced by `/root/.bashrc`), so `docker exec -it px4-ros2-autonomy bash` is enough. Type `rhelp` to list them.

| Shortcut | Runs |
|---|---|
| `cb [args]` | `colcon build --symlink-install` in `/ws`, then re-sources `install/` |
| `cbp PKG...` | same, only `--packages-select PKG...` |
| `cbu PKG...` | same, `--packages-up-to PKG...` (package plus its dependencies) |
| `cclean PKG...` | delete `build/` and `install/` output of those packages |
| `ct` | `colcon test` and print results |
| `ws`, `repo`, `sws` | `cd /ws`, `cd` to this repo, re-source `install/setup.bash` |
| `tl`, `te`, `th`, `ti` | `ros2 topic list / echo / hz / info` |
| `nl`, `ni`, `pl` | `ros2 node list / node info / pkg list` |
| `rl`, `rr` | `ros2 launch`, `ros2 run` |
| `fmu` | list the `/fmu` topics |
| `sim` | `ros2 launch px4_bringup minimal.launch.py` |
| `slam` | `ros2 launch px4_bringup slam.launch.py mode:=mapping` |

Non-interactive commands (`docker exec px4-ros2-autonomy ros2 topic list` without `bash`) do not read `.bashrc`; prefix them with `/entrypoint.sh`.

## Development mode (live edits)

Use this while you change code, so you do not rebuild the 14 GB image for every edit. Start it with the dev override:

```bash
docker compose -f docker-compose.yaml -f docker-compose.dev.yaml up -d
```

What it does:

- Mounts the live repo from the host at `/ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick`, over the copy baked into the image.
- Adds three named volumes, `ws_build`, `ws_install` and `ws_log`, on `/ws/build`, `/ws/install` and `/ws/log`. `colcon build` is therefore incremental (it survives `docker compose down`), and no root-owned build files appear in your repo.

Workflow: **edit on the host** (your editor, git), **build and run inside the container**.

```bash
docker exec -it px4-ros2-autonomy /entrypoint.sh bash
cd /ws && colcon build --symlink-install --packages-select <pkg> && source install/setup.bash
```

With `--symlink-install`, Python edits show up immediately; new files and C++ changes need the `colcon build`.

**Gotcha: stale volumes after an image rebuild.** A named volume is filled from the image only the first time it is created. If you rebuild the image (new dependencies, new PX4 ref, new `px4_msgs`), the old `ws_*` volumes still hold the previous `install/`. Delete them so they are refilled from the new image:

```bash
docker compose -f docker-compose.yaml -f docker-compose.dev.yaml down -v
# or only the three volumes:
docker volume ls | grep ws_            # the names are prefixed with the compose project name
docker volume rm <project>_ws_build <project>_ws_install <project>_ws_log
```

**Baked (non-dev) mode** (`./run.sh` without the override) uses the `install/` inside the image and has no `ws_*` volumes. `./run.sh` rebuilds the image (`up --build`) to pick up code changes.

## VS Code dev container (optional)

`.devcontainer/devcontainer.json` opens this repo inside the dev-mode compose service
(`docker-compose.yaml` + `docker-compose.dev.yaml`), so the editor, terminals, language servers
and linters all run in the container and see ROS, `px4_msgs` and the Python venv.

1. Install the **Dev Containers** extension (`ms-vscode-remote.remote-containers`).
2. Run `xhost +local:docker` on the host (needed for Gazebo/RViz windows).
3. Open the repo folder in VS Code and run **Dev Containers: Reopen in Container**.
4. In a VS Code terminal: `ros2 launch px4_bringup minimal.launch.py`, `colcon build ...` as in dev mode above.

Closing the window stops the compose service (`shutdownAction: stopCompose`). Not tested end to end yet.

## What is persisted

- `./maps` on the host is mounted into the container, so saved SLAM maps and posegraphs survive.
- In dev mode, `/ws/build`, `/ws/install` and `/ws/log` also survive in the `ws_*` volumes (see above).
- Everything else inside the container (PX4 logs, other files you create, and in baked mode `/ws/build` and `/ws/install`) is **lost on `docker compose down`**.

## Paths: avoid hard-coding them

Prefer paths that resolve at runtime, so the same code works on the host and in the container:

- `get_package_share_directory('<pkg>')` / `FindPackageShare` for a package's config, launch and data files.
- The `PX4_DIR` environment variable (set in the image) for the PX4 checkout. `minimal.launch.py` reads it and falls back to the old host path.
- Launch arguments or ROS parameters for anything else (maps, weights, params files).

The `/home/bravo1311/px4_ros2_ws` symlink in the image points at `/ws` only so the existing commands that contain absolute paths keep working. It is a compatibility workaround, not a design to rely on. Once those commands and files use runtime-resolved paths, the symlink can be removed from the `Dockerfile`.

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `unable to find group input` | The image has no named `input` group. Compose uses the numeric GID: set `INPUT_GID` in `.env` (`getent group input`). |
| `/usr/bin/ninja ... failed`, `ninja: command not found` | `ninja-build` must be installed in the final image (it is, in the current Dockerfile). Rebuild if you see this on an old image. |
| `bind error ... port: 8888, errno: 98` | Two XRCE agents are running. Use either `minimal.launch.py` or `docker/start_sim.sh`, not both. |
| RViz clock stays at 0.0 | `/clock` is published by the `clock_bridge` in `perception.launch.py`, which `slam.launch.py` includes. Start `slam.launch.py`; if it exits early, the bridge dies with it. |
| `executable 'aruco_detector' not found` | Rebuild the image: `px4_aruco_landing` now installs the extensionless executables. In dev mode also remove the `ws_*` volumes (see above). |
| `could not select device driver "nvidia"` | NVIDIA Container Toolkit not installed/registered (see Prerequisites). |
| `libEGL warning: ... driver (null)` | Seen in headless runs; harmless. |
