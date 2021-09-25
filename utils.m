%==============================================================================
%:::::::: UTILITIES :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

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
