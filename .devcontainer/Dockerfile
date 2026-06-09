# Dev container for the IPC (Incremental Probabilistic Consensus) SLAM backend.
# Provides every C++ / Python dependency the project needs so nothing is
# installed on the host. g2o is built from source at tag 20201223_git, the
# version the project is pinned to, and placed at the exact path the top-level
# CMakeLists.txt hard-codes — no source edits required.

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

# ---------------------------------------------------------------------------
# System packages
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        ca-certificates \
        sudo \
        gdb \
        # Project C++ deps (IPC fork + g2o baselines)
        libeigen3-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libyaml-cpp-dev \
        # Boost — IPC needs system; the baseline suite also needs
        # filesystem/regex/timer/date_time/program_options. Pull the lot.
        libboost-all-dev \
        # g2o build deps
        libsuitesparse-dev \
        libcholmod3 \
        libmetis-dev \
        # GTSAM tier deps (robust_gtsam / PCM): TBB. MKL is intentionally NOT
        # installed — GTSAM is built against Eigen+TBB (see CMake patch).
        libtbb-dev \
        # Python tooling for scripts/
        python3 \
        python3-pip \
        python3-numpy \
        python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Build g2o (tag 20201223_git) and install to /usr/local — the standard prefix
# that find_package(G2O) (ipc/cmake/FindG2O.cmake) searches, same place we put
# GTSAM and Kimera-RPGO. No hard-coded author path. Headless build — no Qt /
# viewer needed for the IPC executables.
# ---------------------------------------------------------------------------
ENV G2O_PREFIX=/usr/local
RUN mkdir -p /tmp/g2o-src && cd /tmp/g2o-src \
    && git clone https://github.com/RainerKuemmerle/g2o.git . \
    && git checkout 20201223_git \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${G2O_PREFIX} \
        -DG2O_BUILD_APPS=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_USE_OPENGL=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
    && make -j"$(nproc)" \
    && make install \
    && cd / && rm -rf /tmp/g2o-src

# Make sure runtime linker can find g2o shared libs at run time.
RUN echo "${G2O_PREFIX}/lib" > /etc/ld.so.conf.d/g2o.conf && ldconfig

# ===========================================================================
# GTSAM / Kimera-RPGO — the baseline GTSAM tier (gtsam_*, PCM).
# Only needed when building with -DBUILD_GTSAM_BASELINES=ON. They are installed
# to /usr/local so the baseline find_package(GTSAM)/find_package(KimeraRPGO)
# succeed with no -D flags. GTSAM is built against the SYSTEM Eigen (same as
# g2o) to avoid ABI mismatch, and WITHOUT Intel MKL (Eigen+TBB only).
# If the PCM tier ever fails to compile against a newer GTSAM, pin GTSAM_TAG
# to a release known-good with the Kimera-RPGO commit below.
# ===========================================================================
ENV GTSAM_TAG=4.2.0
RUN mkdir -p /tmp/gtsam-src && cd /tmp/gtsam-src \
    && git clone https://github.com/borglab/gtsam.git . \
    && git checkout ${GTSAM_TAG} \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_USE_QUATERNIONS=OFF \
    && make -j"$(nproc)" \
    && make install \
    && cd / && rm -rf /tmp/gtsam-src \
    && ldconfig

# Kimera-RPGO (provides the PCM backend used by gtsam_PCM_2D/3D).
RUN mkdir -p /tmp/kimera-src && cd /tmp/kimera-src \
    && git clone https://github.com/MIT-SPARK/Kimera-RPGO.git . \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j"$(nproc)" \
    && make install \
    && cd / && rm -rf /tmp/kimera-src \
    && ldconfig

# ---------------------------------------------------------------------------
# Optional Python extras for ATE/RPE plots (paper-style metrics)
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir evo

# ---------------------------------------------------------------------------
# Non-root developer user (matches VS Code dev-container convention)
# ---------------------------------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /workspaces/robust_pgo

CMD ["/bin/bash"]
