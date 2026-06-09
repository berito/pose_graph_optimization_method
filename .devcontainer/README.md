# Dev container for IPC

Runs every C++ / Python dependency inside Docker so nothing is installed on the host.

## What's inside

- **Ubuntu 22.04**
- **C++ toolchain:** `build-essential`, `cmake`, `gdb`, `pkg-config`
- **Project deps:** Eigen3, Boost (system), glog, gflags, yaml-cpp, SuiteSparse, METIS
- **g2o** built from source at tag **`20201223_git`** (the version this project is pinned to) and installed to `/home/slam-emix/Workspace/lib/g2o` — the exact path [CMakeLists.txt:5](../CMakeLists.txt#L5) hard-codes, so the project builds with **zero source edits**
- **Python 3** with `numpy`, `matplotlib`, `evo` for [scripts/](../scripts/)
- Non-root `vscode` user with passwordless sudo

## Usage

### VS Code (recommended)

1. Install the **Dev Containers** extension (`ms-vscode-remote.remote-containers`).
2. Open the IPC folder in VS Code.
3. Command palette → *Dev Containers: Reopen in Container*.
4. First build takes ~5–10 min (compiles g2o). Subsequent opens are instant.

### Plain Docker (no VS Code)

```bash
cd .devcontainer
docker build -t ipc-dev -f Dockerfile ..
docker run --rm -it -v "$(pwd)/..":/workspaces/IPC ipc-dev
```

## Building IPC inside the container

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Run an experiment
./ipc_tester_2D -c ../cfg/2D/INTEL_params.yaml
./ipc_tester_3D -c ../cfg/3D/CUBE_params.yaml
```

## Notes

- **Visualization (`visualize: 1` in YAML configs) is intentionally disabled** — the container is headless. Keep `visualize: 0` in any config you run inside. Adding ROS / X-forwarding is out of scope here; add it locally if needed.
- **Datasets:** the YAML configs reference absolute paths under the original author's filesystem. Mount your datasets into the container (edit `devcontainer.json` to add a `mounts:` entry) or place them inside the workspace and point the configs at the in-container path.
- **g2o-viewer / Qt are not installed.** The IPC executables don't need them; if you want `g2o_viewer`, re-build g2o in the Dockerfile with `-DG2O_BUILD_APPS=ON -DG2O_USE_OPENGL=ON` and install Qt5 packages.
