#Sync timestamps (delays in pressing rec button and camera differences)
export PATH=$PATH:/home/manos/.local/bin

mkdir $2

echo 'Running matlab in '$1' to align timestamps'
matlab -nodisplay -nosplash -nodesktop -r "cd('/files/Projects/UnderDev/roboslam/tools/evaluation/EVO'); align_data(\"$1/Results/\"); exit;" | tee "$2/alignment_output.txt"

#evo_config reset
evo_config set plot_seaborn_style whitegrid
evo_config set plot_fontfamily serif plot_fontscale 1.2
evo_config set plot_linewidth 1.0
evo_config set plot_reference_linestyle -
evo_config set plot_figsize 12 7
#evo_config set plot_usetex
#evo_config set plot_axis_marker_scale 0.1
evo_config set plot_pose_correspondences false

#-r=trans_part,rot_part
echo 'Computing ATE: '
yes | evo_ape tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" $3 -r=trans_part --align_origin --save_plot=$2/evo_ape_trans.png --save_results $2/ape.zip  | tee   "$2/evo_output.txt"
#yes | evo_ape tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" -r=angle_deg --save_plot=$2/Results/evo_ape_rot.png  | tee -a "$2/Results/evo_output.txt"

echo 'Computing RPE: '
yes | evo_rpe tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" $3 -r=trans_part --align_origin --save_plot=$2/evo_rpe_trans.png --save_results $2/rpe_t.zip | tee -a "$2/evo_output.txt"
yes | evo_rpe tum "$1/Results/synced_gt_tum.csv" "$1/Results/synced_estimated_poses.csv" -r=angle_deg --save_plot=$2/evo_rpe_rot.png --save_results $2/rpe_r.zip | tee -a "$2/evo_output.txt"

echo 'Computing Trajectory 3D: '
yes | evo_traj tum "$1/Results/synced_estimated_poses.csv" --ref="$1/Results/synced_gt_tum.csv" -v --align_origin --sync --plot_mode=xyz --save_plot=$2/evo_traj.png $1  | tee -a "$2/evo_output.txt"

#yes | evo_traj tum "$1/Results/synced_estimated_poses.csv" --ref="$1/Results/synced_gt_tum.csv" $3   --plot_mode=xyz  #--plot 
