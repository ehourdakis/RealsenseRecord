function align_data(start_delay, end_delay, directory)
%clear all, clf, clc;
fname = directory + "gt_tum.csv";
disp("Loading gt in TUM format: 			" + fname)
load (fname)
% Delete the delays while hitting the optitrack record button
%start_delay = 400;
%end_delay   = 200
ogt = gt_tum(start_delay:end-end_delay,:);
out_gt = ogt;

out_gt(:,1) = ogt(:,1) - ogt(1,1);  
out_gt(:,2:4) = out_gt(:,2:4)-out_gt(1,2:4);

fname = directory + 'synced_gt_tum.csv';
disp("Writing synced gt in TUM: 			" + fname)
dlmwrite(fname, out_gt,'delimiter',' ','newline','pc');

% Rescale the timestamps in realsense camera (fix incosistent realsense HW and optitrack)
fname = directory + 'estimated_poses.csv';
disp("Loading vSLAM estimated poses: 			" + fname)
load (fname);

%comp_scale = round(size(out_gt,1)/size(estimated_poses,1));
%estimated_poses(:,1) = out_gt(1:comp_scale:size(out_gt,1),1);
%estimated_poses(:,1) = out_gt(1:comp_scale:size(out_gt,1)-comp_scale,1);
estimated_poses(:,1) = linspace(0,out_gt(end,1),size(estimated_poses,1));

estimated_poses(:,2:4)= estimated_poses(:,2:4) - estimated_poses(1,2:4);
fname = directory + 'synced_estimated_poses.csv';
disp("Writing synced estimated poses in TUM format: 	" + fname)
dlmwrite(fname, estimated_poses,'delimiter',' ','newline','pc');

disp("Start delay in Matlab: 				" + start_delay)
disp("End delay in Matlab: 				" + end_delay)
