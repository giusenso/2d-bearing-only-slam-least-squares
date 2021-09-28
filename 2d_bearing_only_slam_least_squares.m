clc
clear all
close all

%==============================================================================
%:::::::: IMPORTS :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

addpath './g2o_wrapper'
source './utils.m'
source './least_squares_utils.m'

%==============================================================================
%:::::::: LOAD GROUND TRUTH :::::::::::::::::::::::::::::::::::::::::::::::::::

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
%:::::::: PARSE POSES EDGE ::::::::::::::::::::::::::::::::::::::::::::::::::::

fprintf('\nParsing data... \n');
fprintf('|--> parse pose edges... ');

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
fprintf('Done.\n');

%==============================================================================
%:::::::: PARSE LANDMARK GUESS ::::::::::::::::::::::::::::::::::::::::::::::::

fprintf('|--> parse landmark edges... ');

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
fprintf('Done.\n');
