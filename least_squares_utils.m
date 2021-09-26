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
    J_k = 1./(p_hat(1:2)'*p_hat(1:2))*[-p_hat(2),p_hat(1)];
    Jr = J_k*J_r;
    Jl = J_k*J_l;
endfunction

%==============================================================================

function [H,b,chi_tot] = posesLinearSys(XR, XL, ZR, ass_ZR, kernel_threshold)
    global pose_dim;
    global landmark_dim;
    num_poses = size(XR,3);
    num_landmarks = size(XL,2);
    system_size = pose_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size,system_size);
    b = zeros(system_size,1);
    chi_tot = 0;
    num_measurements = size(ZR,3);
    for (i = 1:num_measurements)
        Omega = eye(6);
        Omega(1:4,1:4) = (1e-5)*Omega(1:4,1:4);
        poseA_id = ass_ZR(1,i);
        poseB_id = ass_ZR(2,i);
        [e,Ja,Jb] = poseErrorAndJacobian(XR(:,:,poseA_id), XR(:,:,poseB_id), ZR(:,:,i));
        chi_tot = chi_tot + e'*Omega*e;

        POSE_A_ID = poseMatrixIndex(poseA_id, num_poses, num_landmarks);
        POSE_B_ID = poseMatrixIndex(poseB_id, num_poses, num_landmarks);

        H(POSE_A_ID:POSE_A_ID+pose_dim-1, POSE_A_ID:POSE_A_ID+pose_dim-1) += ...
        (Ja')*Omega*Ja;
        H(POSE_A_ID:POSE_A_ID+pose_dim-1, POSE_B_ID:POSE_B_ID+pose_dim-1) += ...
        (Ja')*Omega*Jb;
        H(POSE_B_ID:POSE_B_ID+pose_dim-1, POSE_A_ID:POSE_A_ID+pose_dim-1) += ...
        (Jb')*Omega*Ja;
        H(POSE_B_ID:POSE_B_ID+pose_dim-1, POSE_B_ID:POSE_B_ID+pose_dim-1) += ...
        (Jb')*Omega*Jb;

        b(POSE_A_ID:POSE_A_ID+pose_dim-1) += (Ja')*Omega*e;
        b(POSE_B_ID:POSE_B_ID+pose_dim-1) += (Jb')*Omega*e;
    endfor
endfunction

%==============================================================================

function [H,b,chi_tot] = landmarkLinearSys(XR,XL,ZL,associations_ZL,kernel_threshold)
    global system_size;
    global pose_dim;
    global landmark_dim;
    num_poses = size(XR,3);
    num_landmarks = size(XL,2);
    num_measurements = size(ZL,2);
    H = zeros(system_size,system_size);
    b = zeros(system_size,1);
    chi_tot = 0;
    chi = 0;
    for i = 1:num_measurements
        pose_index = associations_ZL(1,i);
        landmark_index = associations_ZL(2,i);
        Xr = XR(:,:,pose_index);
        Xl = XL(:,landmark_index);
        z = ZL(1,i);
        [e,Jr,Jl] = bearingErrorAndJacobian(Xr,Xl,z);
        chi_tot = chi_tot + (e')*e;

        POSE_ID = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        LANDMARK_ID = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

        H(POSE_ID:POSE_ID+pose_dim-1, POSE_ID:POSE_ID+pose_dim-1) += ...
        (Jr')*Jr;
        H(POSE_ID:POSE_ID+pose_dim-1, LANDMARK_ID:LANDMARK_ID+landmark_dim-1) += ...
        (Jr')*Jl;
        H(LANDMARK_ID:LANDMARK_ID+landmark_dim-1, LANDMARK_ID:LANDMARK_ID+landmark_dim-1) += ...
        (Jl')*Jl;
        H(LANDMARK_ID:LANDMARK_ID+landmark_dim-1, POSE_ID:POSE_ID+pose_dim-1) += ...
        (Jl')*Jr;

        b(POSE_ID:POSE_ID+pose_dim-1) += (Jr')*e;
        b(LANDMARK_ID:LANDMARK_ID+landmark_dim-1) += (Jl')*e;
    endfor
endfunction