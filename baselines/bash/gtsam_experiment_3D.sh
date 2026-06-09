#!/bin/bash
path2_datasets="/home/slam-emix/Datasets/BACK_END/3D/"
dataset_list="TUM_FR1_DESK KITTI_05 KITTI_00"
outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="301124"

# GTSAM related solutions
gtsam_opt="GTSAM"
gtsam_list="GNC"
#gtsam_list="HUBER DCS GNC PCM"

path_exec="../build/robust_gtsam"
cd ${path_exec}

for algo in ${gtsam_list}
do
    for dataset in ${dataset_list}
    do
	cfg_file=${path2_datasets}${dataset}"/params.yaml"
        echo ${cfg_file}
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                output_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${gtsam_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                ./gtsam_${algo}_3D ${input_file} ${cfg_file} ${output_traj} &
            done
            jobs
            wait
        done
        echo "Finished "${dataset}
    done
done
