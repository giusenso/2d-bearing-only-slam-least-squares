%==============================================================================
%:::::::: UTILITIES for LEAST SQUARES :::::::::::::::::::::::::::::::::::::::::
%==============================================================================

# apply non-euclidean perturbation to the state [XR,XL]
function [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dX)
    global pose_dim;
    global landmark_dim;
    for(pose_index = 1:num_poses)
        pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        dXR = dX(pose_matrix_index:pose_matrix_index+pose_dim-1,:);
        XR(:,:,pose_index) = v2t(dXR)*XR(:,:,pose_index);
    endfor
    for(landmark_index = 1:num_landmarks)
        landmark_matrix_index = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
        dXL = dX(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
        XL(:,landmark_index) = XL(:,landmark_index) + dXL;
    endfor
endfunction

%==============================================================================

# get angle distance between phi0 and phi1
function deltaPhi = boxMinus(phi0, phi1)
    shifted_phi1 = phi1-phi0;
    if (shifted_phi1<-pi)
        deltaPhi = abs(shifted_phi1+2*pi);
    elseif (shifted_phi1>pi)
        deltaPhi = abs(shifted_phi1-2*pi);
    else
        deltaPhi = abs(shifted_phi1);
    endif
endfunction


%==============================================================================