function angles = gravity_attitude(acc)
    % Estimate attitutde using accelerometer data as observations of
    % earth's gravity field. Can't compute yaw, only pitch and roll.
    %
    % Inputs: acc
    %     acc: Nx3 tri-axial accelerometer readings (ax, ay, az) in m/s^2
    %
    % Outputs: angles
    %     angles: Nx3 euler angles array (roll, pitch, yaw) in rad

    N = size(acc, 1); % get the number of samples

    a = acc./vecnorm(acc,2,2); % normalize accel readings
    ax = a(:,1); ay = a(:,2); az = a(:,3);

    % compute euler angles and store as property
    angles = zeros(N,3);
    angles(:,1) = atan2(-ay, -az);
    angles(:,2) = asin(ax);
    angles(:,3) = 0.0;
end