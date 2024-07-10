function dcm = euler2dcm(angles)
    % Compute Direction Cosine Matrix (DCM) from euler angles
    % Input: angles
    %   Nx3 euler angles array (roll, pitch, yaw) in rad
    % Output: dcm
    %   Nx3x3 DCMs array
    
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
    
    % compute DCMs array
    if (N > 1)
        dcm = zeros(N, 3, 3);
        dcm(:,0,0) = cp*cy;
        dcm(:,1,0) = sr*sp*cy - cp*sy;
        dcm(:,2,0) = cr*sp*cy + sr*sy;
        dcm(:,0,1) = cp*sy;
        dcm(:,1,1) = sr*sp*sy + cr*cy;
        dcm(:,2,1) = cr*sp*sy - sr*cy;
        dcm(:,0,2) = -sp;
        dcm(:,1,2) = sr*cp;
        dcm(:,2,2) = cr*cp;
    else
        dcm = zeros(3, 3);
        dcm(0,0) = cp*cy;
        dcm(1,0) = sr*sp*cy - cp*sy;
        dcm(2,0) = cr*sp*cy + sr*sy;
        dcm(0,1) = cp*sy;
        dcm(1,1) = sr*sp*sy + cr*cy;
        dcm(2,1) = cr*sp*sy - sr*cy;
        dcm(0,2) = -sp;
        dcm(1,2) = sr*cp;
        dcm(2,2) = cr*cp;
    end
end