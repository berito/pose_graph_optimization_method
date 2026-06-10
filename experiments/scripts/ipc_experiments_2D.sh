#!/bin/bash
# ============================================================================
# MINIMAL-EDIT COPY of ipc/bash/ipc_experiments_2D.sh (the IPC author's driver).
# The ONLY changes vs the author's original are marked `# EDIT:` below — they are
# path variables (our repo layout) + dataset_list (M3500 only) + one mkdir so the
# output dir exists (the author's EXP/ folders pre-existed). Everything else — the
# yq param overrides, loop order, parallel `&` + `wait`, rm — is the author's verbatim.
# Run from repo root, inside the devcontainer.
# ============================================================================

cfg_dir="ipc/cfg/2D"                 # EDIT: author used ${path2_datasets}${dataset}/params.yaml
data_dir="experiments/datasets/2D"   # EDIT: author used ${path2_datasets}${dataset}/SPOILED_DATA
out_root="experiments/results/IPC"   # EDIT: author used ${path2_datasets}${dataset}/EXP
ipc_bin="build/ipc/ipc_tester_2D"    # EDIT: author used ../build/ipc_tester_2D

dataset_list="M3500"                 # EDIT: author used "CSAIL FR079 FRH MIT INTEL M3500"
outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"

# G2O related solutiona
#g2o_opt="G2O_IPC_K2"
g2o_opt="G2O_IPC_REC_20"
#exp_date="210823"
#exp_date="311024"
exp_date="110125"

date
for dataset in ${dataset_list}
do
    cfg_file=${cfg_dir}"/"${dataset}"_params.yaml"                                   # EDIT: path only
    for out in ${outliers}
    do
        mkdir -p ${out_root}"/"${dataset}"/"${exp_date}"/"${g2o_opt}"/"${out}        # EDIT: ensure out dir exists
        for run in ${monte_runs}
        do
            input_file=${data_dir}"/"${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"             # EDIT: path only
            output_traj=${out_root}"/"${dataset}"/"${exp_date}"/"${g2o_opt}"/"${out}"/"${run}".TRJ"  # EDIT: path only
            tmp_yaml="./"${g2o_opt}"_"${out}"_"${run}".yaml"
            cp ${cfg_file} ${tmp_yaml}
            yq -i ".dataset=\"$input_file\"" ${tmp_yaml}
            yq -i ".output=\"$output_traj\"" ${tmp_yaml}
            yq -i ".s_factor=10.0" ${tmp_yaml}
            yq -i ".k_buddies=2" ${tmp_yaml}
            yq -i ".use_best_k_buddies=false" ${tmp_yaml}
            yq -i ".use_recovery=true" ${tmp_yaml}
            yq -i ".fast_reject_th=10.64" ${tmp_yaml}
            yq -i ".slow_reject_th=10.64" ${tmp_yaml}
            ${ipc_bin} -c ${tmp_yaml} &                                              # EDIT: binary path only
        done
        jobs
        wait
        rm ./*.yaml
    done
    echo "Finished "${dataset}
done
date
