clc
clear all
close all

%==============================================================================
%:::::::: IMPORTS :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

addpath './g2o_wrapper'
source './utils.m'
source './least_squares_utils.m'

%==============================================================================
%:::::::: LOAD GROUND TRUTH :::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

fprintf('\nLoading g2o ground truth data... ');

# load ground truth dataset
[landmarks, poses, transitions, observations] = loadG2o('dataset/slam2D_bearing_only_ground_truth.g2o');
poses = poses(2:end);
landmarks = landmarks(2:end);
transitions = transitions(2:end);
observations = observations(2:end);

# define some global variables
global num_poses = length(poses);
global num_landmarks = length(landmarks);
global pose_dim = 3;
global landmark_dim = 2;
global sys_size = num_poses*pose_dim + num_landmarks*landmark_dim;
global num_transitions = length(transitions);
global num_observations = length(observations);
global num_measurements = 0;
for (i = 1:num_observations)
	num_measurements = num_measurements + length(observations(i).observation);
endfor

# state initialization
global XR_truth = zeros(3,3,num_poses);
global XL_truth = zeros(2,num_landmarks);

for (i = 1:num_poses)
	XR_truth(:,:,i) = v2t([poses(i).x poses(i).y poses(i).theta]);
	pose_id_2_index(:,i) = poses(i).id;
endfor

# landmark_id_2_index maps the landmark_id to its own index
global landmark_id_2_index = zeros(1,num_landmarks);
for (i = 1:num_landmarks)
	XL_truth(:,i) = [landmarks(i).x_pose landmarks(i).y_pose];
	landmark_id_2_index(:,i) = landmarks(i).id;
endfor
fprintf('Done.\n');

%==============================================================================
%:::::::: LOAD INITIAL GUESS ::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

fprintf('\nLoading g2o initial guess data... ');

# load initial guess dataset
[landmarks, poses, transitions, observations] = loadG2o('dataset/slam2D_bearing_only_initial_guess.g2o');
poses = poses(2:end);
transitions = transitions(2:end);
observations = observations(2:end);

% state initialization
global XR_guess = zeros(3,3,num_poses);
global XL_guess = zeros(2,num_landmarks);

for (i = 1:num_poses)
	XR_guess(:,:,i) = v2t([poses(i).x poses(i).y poses(i).theta]);	# store initial guess
	pose_id_2_index(:,i) = poses(i).id;
endfor
fprintf('Done.\n');

%==============================================================================
%:::::::: PARSE POSES :::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

fprintf('\nParsing data... \n');
fprintf('|--> parse pose edges\n');

global ZR = zeros(3,3,num_transitions);
global ass_ZR = zeros(2,num_transitions);

for (i = 1:num_transitions)
	ZR(:,:,i) = v2t(transitions(i).v);	
	pose_i_id = transitions(i).id_from;
	pose_j_id = transitions(i).id_to;
	pose_i_index = getIndex(pose_id_2_index,pose_i_id);
	pose_j_index = getIndex(pose_id_2_index,pose_j_id);
	ass_ZR(:,i) = [pose_i_index pose_j_index]';
endfor

%==============================================================================
%:::::::: PARSE LANDMARKS :::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

fprintf('|--> parse landmark edges\n');

global ZL = zeros(1,num_measurements);
global ass_ZL = zeros(2,num_measurements);
m = 1;

for (i = 1:num_observations)
	pose_id = observations(i).pose_id;
	pose_index = getIndex(pose_id_2_index,pose_id);
	num_observed = length(observations(i).observation);
	for (j = 1:num_observed)
		landmark_id = observations(i).observation(j).id;
		landmark_index = getIndex(landmark_id_2_index,landmark_id);
		ZL(m) = observations(i).observation(j).bearing;
		ass_ZL(:,m) = [pose_index;landmark_index];
		m = m + 1;
	endfor
endfor
fprintf('Done.\n');

%==============================================================================
%:::::::: LINEAR TRIANGULATION ::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

fprintf('\nLinear triangulation... ');

fprintf('\n|--> compute observations for each landmark\n');
[out,indices] = sort(ass_ZL(2,:));
landmarks_ids_list = unique(out);
observations_per_landmarks = zeros(size(landmarks_ids_list));

% for each landmark get the pair of meaurements with the best parallax
fprintf('|--> compute best measuraments by parallax\n');
fprintf("|--> triangulate\n")
for	(landmark_id = landmarks_ids_list)
	bearing.pose_id = ass_ZL(1,:)(find(ass_ZL(2,:)==landmark_id));
	bearing.val = ZL(find(ass_ZL(2,:)==landmark_id));
	for i = 1:length(bearing.val)
		bearing.pose(:,i) = t2v(XR_guess(:,:,bearing.pose_id(i)));
		bearing.val(i) = bearing.val(i) + bearing.pose(3,i);
		bearing.val(i) = atan2(sin(bearing.val(i)),cos(bearing.val(i)));
	endfor

	observations_per_landmarks(landmark_id) = sum(out==landmark_id);

	box_minus_matrix = zeros(observations_per_landmarks(landmark_id),observations_per_landmarks(landmark_id));
	for (i = 1:observations_per_landmarks(landmark_id)-1)
		for (j = (i+1):observations_per_landmarks(landmark_id))
			box_minus_matrix(i,j) = boxMinus(bearing.val(i),bearing.val(j));
			if (box_minus_matrix(i,j) > pi/2)
				box_minus_matrix(i,j) = pi - box_minus_matrix(i,j);
			endif
		endfor
	endfor
	[pose0_id, pose1_id] = find(box_minus_matrix==max(max(box_minus_matrix)));

	if(observations_per_landmarks(landmark_id)==1)
		XL_guess(:,landmark_id) = bearing.pose(1:2,1)+[cos(bearing.pose(3,1)); sin(bearing.pose(3,1))];
	else
		p00 = bearing.pose(1:2,pose0_id);
		phi0 = bearing.val(pose0_id);
		p01 = p00 + [cos(phi0); sin(phi0)];
		p10 = bearing.pose(1:2,pose1_id);
		phi1 = bearing.val(pose1_id);
		p11 = p10 + [cos(phi1); sin(phi1)];
		XL_guess(:,landmark_id) = getLinesIntersection(p00,p01, p10,p11);
	endif
	clear bearing
endfor
fprintf('Done.\n');

%==============================================================================
%:::::::: RUN GAUSS-NEWTON ::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

% number of iterations
global num_it = 20;

% ENABLE/DISABLE stopping algorithm whenever chi evolution stalls
global cut_exec = true;

% damping coefficient
global damp_coeff = 1e-6;

fprintf("\nRun Gauss-Newton algorithm... ");
[XR, XL, chi_r, chi_l] = ...
	GaussNewtonAlgorithm(XR_guess,XL_guess,ZL,ZR,ass_ZR,ass_ZL,num_it,cut_exec,damp_coeff);
fprintf("\nDone.\n");

%==============================================================================

