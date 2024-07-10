function q = euler2quat(angles)
    % Compute quaternions from euler angles
    % Input: angles
    %   Nx3 euler angles array (roll, pitch, yaw) in rad
    % Output: q
    %   Nx4 normalized quaternions array
    
    % get the number of columns
    N = size(angles, 1);
    
    % get Nx1 euler angles arrays from input array
    roll = angles(:,1);
    pitch = angles(:,2);
    yaw = angles(:,3);
    
    % compute sin and cos of each angle for later use
    sr = sin(0.5*roll);
    cr = cos(0.5*roll);
    sp = sin(0.5*pitch);
    cp = cos(0.5*pitch);
    sy = sin(0.5*yaw);
    cy = cos(0.5*yaw);
    
    % compute the quaternions array
    q = zeros(N, 4);
    q(:,1) = cr.*cp.*cy + sr.*sp.*sy;
    q(:,2) = sr.*cp.*cy - cr.*sp.*sy;
    q(:,3) = cr.*sp.*cy + sr.*cp.*sy;
    q(:,4) = cr.*cp.*sy - sr.*sp.*cy;
    
end

