# A typical command line would be:
#
# ./eval_evo.sh "/data/trajectory/" "/data/trajectory/Results" -sa 

if [ -z "$1" ]
  then
    echo "No first argument supplied, first argument should be data directory"
    return 0;
fi

#evo_config set plot_seaborn_style whitegrid
#evo_config set plot_fontfamily serif plot_fontscale 1.2
#evo_config set plot_linewidth 1.0
#evo_config set plot_reference_linestyle -
#evo_config set plot_figsize 12 7
##evo_config set plot_usetex
#evo_config set plot_axis_marker_scale 0.001
#evo_config set plot_pose_correspondences false
mkdir -p $1/Results/EVO/
yes | evo_ape tum "$1/Results/synced_estimated_poses.csv" "$1/Results/synced_gt_tum.csv" -sa --align_origin --save_plot=$1/Results/EVO/evo_traj.png
