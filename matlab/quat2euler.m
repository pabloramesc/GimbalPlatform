function angles = quat2euler(q)
    % Compute euler angles from quaternions
    % Input: q
    %   Nx4 normalized quaternions array
    % Output: angles
    %   Nx3 euler angles array (roll, pitch, yaw) in rad
    
    % get the number of columns
    N = size(q, 1);
    
    % get Nx1 quaternions arrays from input array
    q0 = q(:,1);
    q1 = q(:,2);
    q2 = q(:,3);
    q3 = q(:,4);
    
    % compute euler angles array
    angles = zeros(N, 3);
    angles(:,1) = atan2(2.0*(q0.*q1 + q2.*q3), (q0.*q0 - q1.*q1 - q2.*q2 + q3.*q3));
    angles(:,2) = asin(2.0*(q0.*q2 - q1.*q3));
    angles(:,3) = atan2(2.0*(q0.*q3 + q1.*q2), (q0.*q0 + q1.*q1 - q2.*q2 - q3.*q3));
    
end
