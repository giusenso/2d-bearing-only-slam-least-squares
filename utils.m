1;

%==============================================================================
%:::::::: UTILITIES :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

function R = rotation2D(theta)
    s = sin(theta);
    c = cos(theta);
    R = [c,-s; s,c];
endfunction

function T = v2t(v)
    T = eye(3);
    T(1:2,1:2) = rotation2D(v(3));
    T(1:2,3) = v(1:2);
endfunction

function v = t2v(A)
    v(1:2,1) = A(1:2,3);
    v(3,1) = atan2(A(2,1),A(1,1));
endfunction

%==============================================================================

function v_idx = poseMatrixIndex(pose_index, num_poses, num_landmarks)
    global pose_dim;
    global landmark_dim;
    if (pose_index>num_poses)
        v_idx = -1;
        return
    endif
    v_idx = 1 + pose_dim*(pose_index-1);
endfunction

function v_idx = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks)
    global pose_dim;
    global landmark_dim;
    if (landmark_index>num_landmarks)
        v_idx=-1;
        return
    endif
    v_idx=1 + (num_poses)*pose_dim + (landmark_index-1) * landmark_dim;
endfunction

%==============================================================================