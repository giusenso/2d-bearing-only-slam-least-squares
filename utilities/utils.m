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

# get index from id
function index = getIndex(id_to_index,id)
    if ( size(find(id_to_index==id),2)==0 )
        index = -1
    else
        index = find(id_to_index==id);
    endif
endfunction

%==============================================================================

# given two 2d-lines (as two pairs of points), find the unique intersection
function ret = getLinesIntersection(p00,p01,p10,p11)
        k0 = (p01(2)-p00(2))/(p01(1)-p00(1));
        k1 = (p11(2)-p10(2))/(p11(1)-p10(1));
        w21 = p00(2) - k0 * p00(1);
        w22 = p10(2) - k1 * p10(1);
        x_coord = (w22-w21)/(k0-k1);
        y_coord = w21 + k0*x_coord;
        ret = [x_coord; y_coord];
endfunction