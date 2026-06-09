# README #

This repo is a collection of the Robust for Pose Graph Optimization algorithm with their examples.

## Prerequisites

- G2O
- Gtsam
- KimeraRPGO
- yaml-cpp
- Eigen3
- Boost
- Python3

## How to build

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DG2O_ROOT=/path/to/g2o \
  -DKimeraRPGO_DIR=/path/to/Kimera-RPGO/install/lib/cmake/KimeraRPGO \
  -DGTSAM_DIR=/path/to/gtsam/install/lib/cmake/GTSAM
make -j$(nproc --all)
```

## How to use

1. Select your favorite dataset pose graph optimization from [here](https://drive.google.com/drive/folders/1bMYu5dSELykgortOJl90zLbYxSAJXZP1?usp=sharing). The dataset has to be in g2o format, if you download them from [here](https://lucacarlone.mit.edu/datasets/) some of them may need conversion.

2. Generate the ground truth file from that dataset using [SE-Sync](https://github.com/david-m-rosen/SE-Sync). Generate in such a way that the output is a text file that lists the poses of the trajectory in the following format (x, y, theta)

3. Spoil the dataset of your choice with the desired number of outliers using:
```
python3 scripts/generateDataset.py -i /path/to/clean/g2o_file -n n_outliers 
```
> > The original version of this script is from the [vertigo](https://github.com/OpenSLAM-org/openslam_vertigo/blob/master/datasets/generateDataset.py) package.

4. Adjust the config file based on the examples of the cfg folder.

```
# Common Params
dataset : "/path/to/spoiled/dataset.g2o"
output : "trajectory.txt" <- Ouput trajectory file;
canonic_inliers : 256 <- Number of correct loop closures;
max_iters: 1000 <- Max iterations for the optimization;

# For G2O
dataset : "/path/to/spoiled/dataset.g2o"
output : "trajectory.txt" <- Ouput trajectory file;
canonic_inliers : 256 <- Number of correct loop closures;
max_iters: 1000 <- Max iterations for the optimization;
inlier_th: 0.5 <- Inlier th (in case of switchable constraint)
switch_prior: 10.0 <- Switchable Prior Weight
maxmix_weight: 1e-1 <- MaxMix Param1
nu_constraint: 0.9 <- MaxMix Param2
nu_nullHypothesis: 1e-5 <- MaxMix Param3

# For GTSAM
inlier_th: 0.9 <- Inlier th (for PCM RPGO)
alpha: 0.99; <- Parameter for M-Estimators

```
5. Run the following command for running the tester for IPC:
```
# For G2O
./build/robust_g2o/g2o_MAXMIX_2D -cfg ./robust_g2o/cfg/params.yaml
./build/robust_g2o/g2o_sc_2D -cfg ./robust_g2o/cfg/params.yaml

# For GTSAM
./build/robust_gtsam/gtsam_DCS_2D ./robust_gtsam/cfg/params.yaml
./build/robust_gtsam/gtsam_GNC_2D ../robust_gtsam/cfg/params.yaml
./build/robust_gtsam/gtsam_PCM_2D ../robust_gtsam/cfg/params.yaml
```
It will produce 2 files: output_name.txt and output_name.PR.
The first file contains the final estimated trajectory, while the latter
contains (precision, recall, average time of convergence).