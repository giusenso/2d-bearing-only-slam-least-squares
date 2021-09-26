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
#   input:
#       Xi: robot pose (3x3 homogeneous matrix)
#       Xj: robot pose (3x3 homogeneous matrix)
#       Z: transform between Xr1 and Xr2
#   output:
#       e: difference between prediction and measurement, vectorized (6x1)
#       Ji: derivative of the error w.r.t. the perturbation of the first pose (6x3)
#       Jj: derivative of the error w.r.t. the perturbation of the second pose (6x3)
function [e,Ji,Jj] = poseErrorAndJacobian(Xi,Xj,Z)
    Ji = zeros(6,3);
    Jj = zeros(6,3);

    Ri = Xi(1:2,1:2);
    Rj = Xj(1:2,1:2);
    ti = Xi(1:2,3);
    tj = Xj(1:2,3);
    Rdot_0 = [0 -1;1 0];

    Z_hat = eye(3);
    Z_hat(1:2,1:2) = Ri'*Rj;
    Z_hat(1:2,3) = Ri'*(tj-ti);
    Z_diff = Z_hat-Z;
    e = [Z_diff(1:2,1); Z_diff(1:2,2); Z_diff(1:2,3)];

    Jj_mat = (Ri')*Rdot_0*Rj;
    Jj(1:4,3) = [Jj_mat(1:2,1); Jj_mat(1:2,2)];
    Jj(5:6,1:2) = Ri';
    Jj(5:6,3) = Ri'*Rdot_0*tj;
    Ji = Ji-Jj;
endfunction;

%==============================================================================

#   input:
#       Xr: robot pose (3x3 homogeneous matrix)
#       Xl: landmark position (2x1)
#       z:  measured bearing of the landmark
#   output:
#       e: difference between prediction and measurement (scalar)
#       Ji: derivative of the error w.r.t. the perturbation of the pose (1x3)
#       Jj: derivative of the error w.r.t. the perturbation of the landmark (1x2)
function [e,Jr,Jl] = bearingErrorAndJacobian(Xr,Xl,z)
    Jr = zeros(1,3);
    Jl = zeros(1,2);
    R = Xr(1:2,1:2);
    t = Xr(1:2,3);
    Rdot_0 = [0 -1; 1 0];
    p_hat = R'*(Xl-t);
    z_hat = atan2(p_hat(2),p_hat(1));
    e = atan2(sin(z_hat-z), cos(z_hat-z));
    J_r = zeros(2,3);
    J_r(1:2,1:2) = -R';
    J_r(1:2,3) = (R')*Rdot_0'*Xl;
    J_l = R';
    J_k = J_atan2(p_hat);
    Jr = J_k*J_r;
    Jl = J_k*J_l;
endfunction

%==============================================================================