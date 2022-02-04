# Evaluate a SLAM trajectory using EVO, in non-interactive mode
# A typical command line would be:
# ./eval_evo.sh "/data/trajectory/"
export PATH=$PATH:/home/${USER}/.local/bin

SCRIPT_DIR="/files/evaluation/EVO"

echo 'Running matlab in '$1' to align timestamps'
matlab -nodisplay -nosplash -nodesktop -r "cd("$SCRIPT_DIR"); align_data(\"$1/\"); exit;"

evo_config set plot_seaborn_style whitegrid
evo_config set plot_fontfamily serif plot_fontscale 1.2
evo_config set plot_linewidth 1.0
evo_config set plot_reference_linestyle -
evo_config set plot_figsize 12 7
#evo_config set plot_usetex
#evo_config set plot_axis_marker_scale 0.1
evo_config set plot_pose_correspondences false

evo_ape tum "$1/synced_estimated_poses.csv" "$1/synced_gt_tum.csv"  -sa -r=trans_part  --plot --save_results $1/out.zip
#evo_rpe tum "$1/synced_estimated_poses.csv" "$1/synced_gt_tum.csv" -as -r=angle_deg --plot
#evo_traj tum "$1/synced_estimated_poses.csv" --ref="$1/synced_gt_tum.csv" -v -sa --align_origin --sync --plot
#evo_traj tum "$1/synced_estimated_poses.csv" --ref="$1/synced_gt_tum.csv" -a -v --align_origin --sync --plot
