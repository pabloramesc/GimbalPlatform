function angles = accel_magnet_attitude(acc, mag)
    % Estimate attitutde using accelerometer and magnetometer data.
    % With accelerometer tilt is calculated (roll and pitch), then
    % calibrated magnetometer data is rotated and yaw is computed.
    %
    % Inputs: acc, mag, mag_bias
    %     acc: Nx3 tri-axial accelerometer readings (ax, ay, az) in m/s^2
    %     mag: Nx3 tri-axial magnetometer readings (mx, my, mz) in gauss
    %
    % Outputs: angles
    %     angles: Nx3 euler angles array (roll, pitch, yaw) in rad

    a = acc./vecnorm(acc,2,2); % normalize accel readings
    ax = a(:,1); ay = a(:,2); az = a(:,3);

    % compute roll and pitch from gravity
    roll = atan2(-ay, -az);
    pitch = atan2(ax, sqrt(ay.^2 + az.^2));
    
    m = mag./vecnorm(mag,2,2); % normalize magnet readings
    mx = m(:,1); my = m(:,2); mz = m(:,3);
    
    % compute sin and cos of roll and pitch for later use
    sr = sin(roll); cr = cos(roll);
    sp = sin(pitch); cp = cos(pitch);
    
    % rotate magnetometer data to the horizontal plane
    rx = mx.*cp + my.*sr.*sp + mz.*cr.*sp;
    ry = my.*cr - mz.*sr;
    
    % compute yaw as the tilt-compensated heading angle to Magnetic North
    yaw = atan2(-ry, rx);
    
    angles = [roll pitch yaw];    
    
end