#!/bin/bash

# Evaluate a SLAM trajectory using EVO
# Sync timestamps (delays in pressing rec button and camera differences)
# Expects 3 arguments:
# 1: Directory of output trajectory 
# 2: Directory to store the results of the evaluation
# 3: What trajectory processing EVO should perform (use -sa for scaling and aligning the trajectories)
# 
# A typical command line would be:
#
# ./eval_evo.sh "/data/trajectory/" "/data/trajectory/Results" -sa 

if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data directory"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

# Goto to data dir
cd $REALSENSE_RECORD_DIR/scripts/evaluation/EVO

# Sychronize data
octave-cli align_data_octave.m $1/Results

export PATH=$PATH:/home/${USER}/.local/bin

# Configure EVO drawing. This has to run only once.
#evo_config reset
evo_config set plot_seaborn_style whitegrid
evo_config set plot_fontfamily serif plot_fontscale 1.2
evo_config set plot_linewidth 1.0
evo_config set plot_reference_linestyle -
evo_config set plot_figsize 12 7
#evo_config set plot_usetex
#evo_config set plot_axis_marker_scale 0.1
evo_config set plot_pose_correspondences false

mkdir -p $1/Results/EVO/
# Evaluate the ATE and RPE for the given trajectory
echo 'Computing ATE: '
yes | evo_ape tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" $2 -r=trans_part --align_origin --save_plot=$1/Results/EVO/evo_ape_trans.png --save_results $1/Results/EVO/ape.zip  | tee   "$1/Results/EVO/evo_output.txt"
yes | evo_ape tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" -r=angle_deg --save_plot=$1/Results/EVO/evo_ape_rot.png  | tee -a "$1/Results/EVO/evo_output.txt"

echo 'Computing RPE: '
yes | evo_rpe tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" $2 -r=trans_part --align_origin --save_plot=$1/Results/EVO/evo_rpe_trans.png --save_results $1/Results/EVO/rpe_t.zip | tee -a "$1/Results/EVO/evo_output.txt"
yes | evo_rpe tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" -r=angle_deg --save_plot=$1/Results/EVO/evo_rpe_rot.png --save_results $1/Results/EVO/rpe_r.zip | tee -a "$1/Results/EVO/evo_output.txt"

echo 'Computing Trajectory 3D: '
yes | evo_traj tum "$1/Results/synced_estimated_poses.csv" --ref="$1/Results/synced_gt_tum.csv" $2 -v --align_origin --sync --plot_mode=xyz --save_plot=$1/Results/EVO/evo_traj.png | tee -a "$1/Results/EVO/evo_output.txt"

cd ${cwd}