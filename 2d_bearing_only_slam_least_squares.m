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
global system_size = num_poses*pose_dim + num_landmarks*landmark_dim;
global num_observations = length(observations);

# state initialization
global XR_truth = zeros(3,3,num_poses);
global XL_truth = zeros(2,num_landmarks);

# maps pose id to index
global pose_id_2_index = zeros(1,num_poses);
for (i = 1:num_poses)
	XR_truth(:,:,i) = v2t([poses(i).x poses(i).y poses(i).theta]);
	pose_id_2_index(:,i) = poses(i).id;
endfor

# maps landmark id to index
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
