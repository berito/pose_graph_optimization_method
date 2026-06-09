#!/bin/bash

path2_datasets="/home/slamemix/Data/RobustSLAM/2D/"
#path2_datasets="/home/slamemix/Data/RobustSLAM/3D/"

dataset_list="INTEL CSAIL FR079 FRH"
#dataset_list="TUM_FR1_DESK KITTI_05 KITTI_00"

outliers="10 20 30 40 50"
#outliers="60 70 80 90"
monte_runs="01 02 03 04 05 06 07 08 09"

date="260214"

# G2O related solutiona
g2o_opt="G2O"
g2o_list="HUBER DCS MAXMIX RRR SC GNC TACO IPC_S10"

# CLASSIC METRICS FOR EVALUATION
for algo in ${g2o_list}
do
    for dataset in ${dataset_list}
    do
        gt_traj=${path2_datasets}${dataset}"/GT.txt"
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                output_res=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TE" 
                ../build/evaluator/evaluator ${input_traj} ${gt_traj} ${output_res} &
            done
            wait
        done
        echo "Finished "${dataset}
    done
done