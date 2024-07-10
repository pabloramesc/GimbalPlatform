function q = complementary_filter(acc, mag, gyr, q0, Dt, gain)
    % Estimate attitude by fusing accelerometer and gyroscope estimations
    % with a complementary filter (linear interpolator).
    %
    % Inputs: acc, gyr, q0, Dt, gain
    %     acc: Nx3 tri-axial accelerometer readings (ax, ay, az) in m/s^2
    %     mag: Nx3 tri-axial magnetometer readings (mx, my, mz) in gauss
    %     gyr: Nx3 tri-axial gyroscope readings (wx, wy, wz) in rad/s
    %     q0: 1x4 initial attitude as quaternion
    %     Dt: N sampling periods array in seconds
    %     gain: gain of filter as float in [0.0, 1.0]. If gain = 1.0 the
    %     filter will only use the accelerometer and magnetometer data.
    %     If gain = 0.0 the filter will only use gyroscope data.
    %
    % Outputs: q
    %     q: Nx4 normalized quaternions array with the attitude
    
    N = size(gyr, 1); % get the number of samples
    a = acc./vecnorm(acc,2,2); % normalize accel readings
    m = mag./vecnorm(mag,2,2); % normalize magnet readings
    
    q = zeros(N,4);
    q(1,:) = q0;
    for k = 2:N
        % Step 1) Attitude propagation integrating gyro
        dt = Dt(k); % current sampling period
        wx = gyr(k,1); wy = gyr(k,2); wz = gyr(k,3);
        % compute Omega matrix
        Omega = [ 0.0  -wx  -wy  -wz
                  +wx  0.0  +wz  -wy
                  +wy  -wz  0.0  +wx
                  +wz  +wy  -wx  0.0 ];
        q_prev = q(k-1,:)'; % get las quaternion as column (4x1)
        A = eye(4) + 0.5*Omega*dt;
        q_gyr = A*q_prev;
        q_gyr = q_gyr/norm(q_gyr); % normalize the quaternion
        q_gyr = q_gyr'; % quaternion as row (1x4)
        
        % Step 2) Compute attitude with accel and magnet data
        % compute roll and pitch from gravity
        ax = a(k,1); ay = a(k,2); az = a(k,3);
        roll = atan2(-ay, -az);
        pitch = asin(ax);
        % rotate magnetometer data and compute yaw
        mx = m(k,1); my = m(k,2); mz = m(k,3);
        sr = sin(roll); cr = cos(roll);
        sp = sin(pitch); cp = cos(pitch);
        rx = mx*cp + my*sr*sp + mz*cr*sp;
        ry = my*cr - mz*sr;
        yaw = atan2(-ry, rx);
        q_acc = euler2quat([roll pitch yaw]);
        
        % Step 3) Combine each estimation with the complementary filter
        q(k,:) = (1-gain)*q_gyr + gain*q_acc; % store the computed quaternion
    end
    
end