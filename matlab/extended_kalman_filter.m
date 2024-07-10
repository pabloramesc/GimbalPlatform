function q = extended_kalman_filter(acc, mag, gyr, q0, Dt, noises)
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
    %     filter only use the accelerometer data. If gain = 0.0 the filter
    %     only use gyroscope data.
    %
    % Outputs: q
    %     q: Nx4 normalized quaternions array with the attitude
    
    % Prediction
    % 1) x_est = F * x_prev
    % 2) P_est = F * P_prev * F' + Q
    
    % Correction
    % 3) y_est = h(x_est)
    % 4) K = P_est * H' * inv(H * P_est * H' + R)
    % 5) x = x_est + K * (y_meas - y_est)
    % 6) P = P_est - K * H * P_est
    
    % get sensors noises
    sig_a = noises(1);
    sig_m = noises(2);
    sig_g = noises(3);
    
    N = size(gyr, 1); % get the number of samples
    a = acc./vecnorm(acc,2,2); % normalize accel readings
    m = mag./vecnorm(acc,2,2); % normalize magnet readings
    
    % build measurement error covariance matrix
    R = diag([sig_a^2 sig_a^2 sig_a^2 sig_m^2 sig_m^2 sig_m^2]);
    
    q = zeros(N,4);
    q(1,:) = q0;
    x_prev = q0';
    P_prev = eye(4);
    for k = 2:N
        
        dt = Dt(k); % current sampling period
        ax = a(k,1); ay = a(k,2); az = a(k,3);
        mx = m(k,1); my = m(k,2); mz = m(k,3);
        wx = gyr(k,1); wy = gyr(k,2); wz = gyr(k,3);
        qw = x_prev(1); qx = x_prev(2); qy = x_prev(3); qz = x_prev(4);

        % Step 1) compute estate prediction      
        % compute F Jacobian matrix
        Omega = [ 0.0  -wx  -wy  -wz
                  +wx  0.0  +wz  -wy
                  +wy  -wz  0.0  +wx
                  +wz  +wy  -wx  0.0 ];
        F = 0.5*Omega*dt + eye(4);
        x_est = F*x_prev; % prediction
        x_est = x_est/norm(x_est); % normalize the quaternion
        
        % Step 2) compute prediction covariance
        % compute the process covariance matrix
        W = 0.5*dt * [-qx  -qy  -qz
                      +qw  -qz  +qy
                      +qz  +qw  -qx
                      -qy  +qx  +qw];
        Q = sig_g^2 * W * W';
        P_est = F * P_prev * F' + Q;
        
        % Step 3) estimate measurement 
        y_est = [-2*qx*qz + 2*qy*qw
                 -2*qy*qz - 2*qx*qw
                 -qw^2 + qx^2 + qy^2 - qz^2
                  qw^2 + qx^2 - qy^2 - qz^2
                  2*qx*qy - 2*qz*qw
                  2*qx*qz + 2*qy*qw];
        
        % Step 4) compute Kalman gain
        % build measurement model Jacobian matrix
        H = [+2*qy  -2*qz  +2*qw  -2*qx
             -2*qx  -2*qw  -2*qz  -2*qy
             -2*qw  +2*qx  +2*qy  -2*qz
             +2*qw  +2*qx  -2*qy  -2*qz
             -2*qz  +2*qy  +2*qx  -2*qw
             +2*qy  +2*qz  +2*qw  +2*qx];
        K = P_est * H' * inv(H * P_est * H' + R);
        
        
        % Step 5) correct predicted state and its covariance
        y_meas = [ax; ay; az; mx; my; mz];
        x = x_est + K * (y_meas - y_est);
        x = x/norm(x); % normalize the quaternion
        P = P_est - K * H * P_est;
        
        % Store data
        P_prev = P;
        x_prev = x;
        q(k,:) = x'; % quaternion as row (1x4)
        
    end
       
end