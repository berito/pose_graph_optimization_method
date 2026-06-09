# Dependencies

This monorepo builds two halves from one superbuild (`CMakeLists.txt` at the root):

- **`ipc/` + `baselines/robust_g2o` + `baselines/evaluator`** — C++14/17, need only
  **g2o + glog + yaml-cpp + Boost + Eigen**. This is the default build.
- **`baselines/robust_gtsam` (GTSAM tier incl. PCM)** — additionally needs
  **GTSAM + Kimera-RPGO + TBB**. Opt-in via `-DBUILD_GTSAM_BASELINES=ON`.

Everything below is installed automatically by `.devcontainer/` — read on only if you build
on the host. Install the C++ libraries first; the rest is wired through CMake.

## System packages (Ubuntu 20.04 / 22.04 / 24.04)

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libeigen3-dev \
    libboost-all-dev \
    libgoogle-glog-dev libgflags-dev \
    libyaml-cpp-dev \
    libsuitesparse-dev libcholmod3 \
    qtbase5-dev libqglviewer-dev-qt5 \
    python3 python3-pip python3-numpy python3-matplotlib
```

`libsuitesparse-dev` and the Qt packages are pulled in transitively by g2o for the CHOLMOD solver and the optional GUI. Drop the Qt packages if you don't need `g2o_viewer`.

## g2o (built from source — required)

The project is pinned to g2o tag `20201223_git`. The packaged `libg2o` in Ubuntu is generally too old or built without the needed types. The IPC build has `set(G2O_ROOT /home/slam-emix/Workspace/lib/g2o)` at [ipc/CMakeLists.txt:5](ipc/CMakeLists.txt#L5) — **edit this to your install prefix, or delete the line so [ipc/cmake/FindG2O.cmake](ipc/cmake/FindG2O.cmake) searches system paths**.

```bash
cd ~/Workspace/lib   # or wherever you want it
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20201223_git
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=$HOME/Workspace/lib/g2o/install
make -j$(nproc)
make install
```

Then set `G2O_ROOT` in [CMakeLists.txt](CMakeLists.txt) to the `install/` directory above (or export it as an env var and remove the hard-coded line).

## Python (for the scripts/ utilities)

`scripts/generateDataset.py` and `scripts/plotATERPET.py` use the standard scientific stack:

```bash
python3 -m pip install --user numpy matplotlib argparse evo
```

`evo` is optional — only needed if you want absolute trajectory error (ATE) / relative pose error (RPE) plots comparable to the paper.

## Optional: ROS (only if `visualize: 1` in a config)

The codebase publishes to RViz when visualization is enabled. If you don't use it, leave `visualize: 0` in your YAML and ROS is not required. If you do:

```bash
sudo apt install -y ros-noetic-rviz   # or your ROS distro
```

and source the appropriate `setup.bash` before running.

## GTSAM tier (optional — for `baselines/robust_gtsam`, incl. PCM)

Only needed when building with `-DBUILD_GTSAM_BASELINES=ON`. The `.devcontainer/Dockerfile`
builds these from source into `/usr/local`; the manual recipe:

```bash
sudo apt install -y libtbb-dev libboost-all-dev   # TBB + full Boost (filesystem/regex/...)

# GTSAM (built against system Eigen, no MKL — Eigen+TBB only)
git clone https://github.com/borglab/gtsam.git && cd gtsam && git checkout 4.2.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc) && sudo make install && sudo ldconfig

# Kimera-RPGO (provides the PCM backend)
git clone https://github.com/MIT-SPARK/Kimera-RPGO.git && cd Kimera-RPGO
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig
```

Intel MKL is intentionally **not** required (our CMake patch makes it optional).

## Build sanity check

```bash
cd /home/lab_desktop/Documents/code_base/thesis/robust_pgo
mkdir -p build && cd build

# Default: IPC + g2o baselines + evaluator
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./ipc_tester_2D -c ../cfg/2D/INTEL_params.yaml      # IPC still runs
ls IN_SC_2D IN_MAXMIX_2D IN_DCS_2D evaluator         # baselines built

# Add the GTSAM/PCM tier (after installing GTSAM + Kimera-RPGO above)
cmake .. -DBUILD_GTSAM_BASELINES=ON && make -j$(nproc)
ls gtsam_PCM_2D
```

If `find_package(G2O)` fails, the most common cause is `G2O_ROOT` pointing somewhere wrong — see the g2o section above. If the GTSAM tier fails to find GTSAM/Kimera, ensure both installed to a prefix CMake searches (e.g. `/usr/local`) or pass `-DGTSAM_DIR=…` / `-DKimeraRPGO_DIR=…`.

## Claude agent

A project subagent has been written to [.claude/agents/ipc-slam-expert.md](.claude/agents/ipc-slam-expert.md). Invoke it from Claude Code with:

```
/agents
```

…or just describe an IPC-related task and the harness will auto-route to it based on the agent's `description:` field.
