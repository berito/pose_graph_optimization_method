#!/bin/bash
path2_datasets="/home/${USER}/Datasets/BACK_END/2D/"
dataset_list="CSAIL MIT FR079 FRH INTEL"
outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="251202"

# GTSAM related solutions
gtsam_opt="GTSAM"
gtsam_list="HUBER DCS GNC"
#gtsam_list="HUBER DCS GNC PCM"

command_prefix="./build/robust_gtsam"
gtsam_config="./robust_gtsam/cfg/params.yaml"
tmp_folder="./build/robust_gtsam/tmp"

for algo in ${gtsam_list}
do
    for dataset in ${dataset_list}
    do
	    dataset_cfg_file=${path2_datasets}${dataset}"/params.yaml"
        n_inliers=$(yq '.canonic_inliers' ${dataset_cfg_file})
        gt_file=$(yq '.ground_truth' ${dataset_cfg_file})
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                output_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${gtsam_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                tmp_yaml=${tmp_folder}"/"${gtsam_opt}"_"${algo}"_"${out}"_"${run}".yaml"
                cp ${gtsam_config} ${tmp_yaml}
                yq -i ".name=\"$dataset\"" ${tmp_yaml}
                yq -i ".dataset=\"$input_file\"" ${tmp_yaml} 
                yq -i ".ground_truth=\"$gt_file\"" ${tmp_yaml}
                yq -i ".output=\"$output_traj\"" ${tmp_yaml}
                yq -i ".canonic_inliers=\"$n_inliers\"" ${tmp_yaml}
                ${command_prefix}/gtsam_${algo}_2D ${tmp_yaml} &
            done
            jobs
            wait
            rm ${tmp_folder}"/*"
        done
        echo "Finished "${dataset}
    done
done
