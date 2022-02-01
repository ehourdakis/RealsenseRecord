echo 'Using previously aligned timestamps (from EVO)'
#echo 'Running matlab in '$3' to align timestamps'
#matlab -nodisplay -nosplash -nodesktop -r "cd('/files/Projects/UnderDev/roboslam/tools/evaluation/EVO'); align_data($1, $2, \"$3\"); exit;"

echo 'Computing Original ATE: '
python2 original_ate.py "$3/synced_estimated_poses.csv" "$3/synced_gt_tum.csv"  --scale $4 --verbose # --plot "$3/ate.png"

#This is the evaluate 2 (dont remember where i got it) that includes 3D plot
echo 'Computing Evaluate2 ATE: '
python2 evaluate2.py "$3/synced_estimated_poses.csv" "$3/synced_gt_tum.csv" --scale $4 --verbose --plot3D

#This is the relative rpe error
echo 'Computing RPE: '
python2 original_rpe.py "$3/synced_gt_tum.csv" "$3/synced_estimated_poses.csv" --scale $4 --verbose --fixed_delta # --plot 1.png
