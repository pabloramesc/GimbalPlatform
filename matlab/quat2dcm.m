function dcm = quat2dcm(q)
    % Compute Direction Cosine Matrix (DCM) from quaternion
    % Input: q
    %   Nx4 normalized quaternions array
    % Output: dcm
    %   Nx3x3 DCMs array
    
    % get the number of columns
    N = size(q, 1);
    
    if (N > 1)
        % get Nx1 quaternions arrays from input array
        q0 = q(:,1);
        q1 = q(:,2);
        q2 = q(:,3);
        q3 = q(:,4);
        % compute DCMs matrix array
        dcm = zeros(N, 3, 3);
        dcm(:,1,1) = 1.0 - 2.0*(q2.*q2 + q3.*q3);
        dcm(:,2,1) = 2.0*(q1.*q2 + q0.*q3);
        dcm(:,3,1) = 2.0*(q1.*q3 - q0.*q2);
        dcm(:,1,2) = 2.0*(q1.*q2 - q0.*q3);
        dcm(:,2,2) = 1.0 - 2.0*(q1.*q1 + q3.*q3);
        dcm(:,3,2) = 2.0*(q0.*q1 + q2.*q3);
        dcm(:,1,3) = 2.0*(q1.*q3 + q0.*q2);
        dcm(:,2,3) = 2.0*(q2.*q3 - q0.*q1);
        dcm(:,3,3) = 1.0 - 2.0*(q1.*q1 + q2.*q2);
    else
        % get quaternion components
        q0 = q(1);
        q1 = q(2);
        q2 = q(3);
        q3 = q(4);
        % compute DCM matrix
        dcm = zeros(3, 3);
        dcm(1,1) = 1.0 - 2.0*(q2.*q2 + q3.*q3);
        dcm(2,1) = 2.0*(q1.*q2 + q0.*q3);
        dcm(3,1) = 2.0*(q1.*q3 - q0.*q2);
        dcm(1,2) = 2.0*(q1.*q2 - q0.*q3);
        dcm(2,2) = 1.0 - 2.0*(q1.*q1 + q3.*q3);
        dcm(3,2) = 2.0*(q0.*q1 + q2.*q3);
        dcm(1,3) = 2.0*(q1.*q3 + q0.*q2);
        dcm(2,3) = 2.0*(q2.*q3 - q0.*q1);
        dcm(3,3) = 1.0 - 2.0*(q1.*q1 + q2.*q2);
    end
    
end