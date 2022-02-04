% Align the time-series of the data, by finding the first time instant that the derivative on each axis of motion 
% The function expects a directory that contains a timestamped ground truth file (gt.csv) and a timestamped pose
% file with the SLAM poses. 
% Poses are expected as a 3D vector for position, 4D vector for quaternioned
function align_data(directory)

%% Load Original gt file
fname = directory + "/gt.csv";
disp("Loading gt: 	         	         	" + fname)
load (fname)

%% Convert to the TUM convention
gt_tum(:,1)   = gt(:,2);
gt_tum(:,2:4) = gt(:,7:9);
gt_tum(:,5:8) = gt(:,3:6);

%% Find zero-crossings (indicating motion) in any axis and set the starting point
x = gt_tum(:,1);

fx = gt_tum(:,2);
px=findchangepts(fx,'Statistic','linear','MinThreshold',0.0001);
fy = gt_tum(:,3);
py=findchangepts(fy,'Statistic','linear','MinThreshold',0.0001);
fz = gt_tum(:,4);
pz=findchangepts(fz,'Statistic','linear','MinThreshold',0.0001);

gt_start = max(min(px(1), py(1)),pz(1));
gt_end   = min(max(px(end), py(end)),pz(end));

% Plot the points being removed
ogt = gt_tum(gt_start:gt_end,:);
%subplot(2,1,2), plot(ogt(:,1),ogt(:,2));

%% Recenter to time 0 and pose 0 after transformations
out_gt = ogt;
out_gt(:,1) = ogt(:,1) - ogt(1,1);  
out_gt(:,2:4) = out_gt(:,2:4)-out_gt(1,2:4);

%% Save the synchronized trajectories
fname = directory + '/synced_gt_tum.csv';
disp("Writing synced gt in TUM: 			" + fname)
dlmwrite(fname, out_gt,'delimiter',' ','newline','pc');

%% Rescale the timestamps in realsense camera (fix incosistent realsense HW and optitrack)
fname = directory + '/estimated_poses.csv';
if ~isfile(fname) 
	disp("Trajectory has not been processed.")
	exit;
end
disp("Loading vSLAM estimated poses: 			" + fname)
load (fname);

%% Find zero-crossings (indicating motion) in any axis and set the starting point
x = estimated_poses(:,1);

fx = estimated_poses(:,2);
px=findchangepts(fx,'Statistic','linear','MinThreshold',0.0001);
fy = estimated_poses(:,3);
py=findchangepts(fy,'Statistic','linear','MinThreshold',0.0001);
fz = estimated_poses(:,4);
pz=findchangepts(fz,'Statistic','linear','MinThreshold',0.0001);

est_start = max(min(px(1), py(1)),pz(1));
est_end   = min(max(px(end), py(end)),pz(end));

est = estimated_poses(est_start:est_end,:);

%% Match the trajectory timestamps using linear interposlation on the resized time-series
estimated_poses = est;
estimated_poses(:,1) = linspace(0,out_gt(end,1),size(estimated_poses,1));
estimated_poses(:,2:4)= estimated_poses(:,2:4) - estimated_poses(1,2:4);
%subplot(2,1,2), plot(est(:,1),est(:,2));

%% Write results
fname = directory + '/synced_estimated_poses.csv';
disp("Writing synced estimated poses in TUM format: 	" + fname)
dlmwrite(fname, estimated_poses,'delimiter',' ','newline','pc');

%% Cout results
disp("gt Start delay in Matlab:   " + gt_start)
disp("gt End delay in Matlab:     " + gt_end)
disp("est Start delay in Matlab:  " + est_start)
disp("est End delay in Matlab:    " + est_end)
