#!/bin/bash
path2_datasets="/home/slamemix/Data/RobustSLAM/3D/"
dataset_list="KITTI_00 KITTI_05 TUM_FR1_DESK"
outliers="10 20 30 40 50"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="260214"

# G2O related solutiona
g2o_opt="G2O"
g2o_list="DCS MAXMIX GNC HUBER SC RRR"

cd /home/slamemix/Workspace/RPGO/RobustOptimizationSLAM/build/robust_g2o/src/incr
config="/home/slamemix/Workspace/RPGO/RobustOptimizationSLAM/robust_g2o/cfg/params_3D.yaml"
tmp_folder="tmp"

for algo in ${g2o_list}
do
    for dataset in ${dataset_list}
    do
        cfg_file=${path2_datasets}${dataset}"/params.yaml"
        cp_params=./${tmp_folder}/params.yaml
        cp ${cfg_file} ${cp_params}
        n_inliers=$(yq '.canonic_inliers' ${cp_params})
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                output_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                tmp_yaml=${tmp_folder}"/"${g2o_opt}"_"${algo}"_"${out}"_"${run}".yaml"
                cp ${config} ${tmp_yaml}
                yq -i ".name=\"$dataset\"" ${tmp_yaml}
                yq -i ".dataset=\"$input_file\"" ${tmp_yaml} 
                yq -i ".output=\"$output_traj\"" ${tmp_yaml}
                yq -i ".canonic_inliers=\"$n_inliers\"" ${tmp_yaml}
                ./IN_${algo}_3D -cfg ${tmp_yaml} &
            done
            jobs
            wait
            rm ./${tmp_folder}/*.yaml
        done
        echo "Finished "${dataset}
    done
done

