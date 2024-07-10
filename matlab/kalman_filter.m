function q = kalman_filter(acc, mag, gyr, q0, Dt, noises)
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
    % 1) x_est = F * x_est_prev
    % 2) P_est = F * P_prev * F' + Q_prev
    
    % Correction
    % 3) y_est = H * x_est
    % 4) z = y_meas - y_est
    % 5) K = P_est * H' * inv(H * P_est * H' + R) 
    % 6) x_corr = x_est + K * z
    % 7) P = P_est - K * H * P_est
    
    % get sensors noises
    acc_noise = noises(1);
    mag_noise = noises(2);
    gyr_noise = noises(3);
    
    % measurement noise covariance matrix
    R = diag([acc_noise^2 acc_noise^2 acc_noise^2 mag_noise^2 mag_noise^2 mag_noise^2]);
    
    N = size(gyr, 1); % get the number of samples
    a = acc./vecnorm(acc,2,2); % normalize accel readings
    m = mag./vecnorm(mag,2,2); % normalize magnet readings
    
    % global references (earth's gravitational and magnetic field)
    g = [0 0 1]; % gravity
    gx = g(1); gy = g(2); gz = g(3);
    dip = deg2rad(-0.05); % Madrid magnetic declination
    r = 1/sqrt(cos(dip)^2+sin(dip)^2)*[cos(dip) 0 sin(dip)];
    rx = r(1); ry = r(2); rz = r(3);
    
    q = zeros(N,4);
    q(1,:) = q0;
    Pt_ = eye(4);
    for k = 2:N
        
        dt = Dt(k); % current sampling period
        ax = a(k,1); ay = a(k,2); az = a(k,3);
        mx = m(k,1); my = m(k,2); mz = m(k,3);
        wx = gyr(k,1); wy = gyr(k,2); wz = gyr(k,3);

        % Step 1) integrate gyro to predict new quaternion        
        % compute Omega matrix
        Omega = [ 0.0  -wx  -wy  -wz
                  +wx  0.0  +wz  -wy
                  +wy  -wz  0.0  +wx
                  +wz  +wy  -wx  0.0 ];
        qt_prev = q(k-1,:)'; % get las quaternion as column (4x1)
        A = eye(4) + 0.5*Omega*dt;
        qt = A*qt_prev;
        qt = qt/norm(qt); % normalize the quaternion
        
        % Step 2) Compute prediction error
        % compute gradient matrix F
        Ft = [1.0       -dt/2*wx  -dt/2*wy  -dt/2*wz
              +dt/2*wx  1.0       +dt/2*wz  -dt/2*wy
              +dt/2*wy  -dt/2*wz  1.0       +dt/2*wx
              +dt/2*wz  +dt/2*wy  -dt/2*wx  1.0     ];
        % compute the process covariance matrix
        qw = qt_prev(1); qx = qt_prev(2); qy = qt_prev(3); qz = qt_prev(4);
        Wt = dt/2 * [-qx  -qy  -qz
                     +qw  -qz  +qy
                     +qz  +qw  -qx
                     -qy  +qx  +qw];
        % process noise covariance matrix
        Qt = gyr_noise^2 * (Wt * Wt');
        % compute predicted error
        Pt = Ft * Pt_ * Ft' + Qt;
        
        % Step 3) Compute innovation with measurements
        zt = [ax ay az mx my mz]'; % measurement vector (6x1)
        % compute the measuremente model
        Rot = quat2dcm(qt'); % rotation matrix
        ra = Rot*g'; rm = Rot*r'; % rotate gravity and magnetic field
        ht = cat(1, ra, rm); % build measurement model
        vt = zt - ht;
        
        % Step 4) Compute correction error
        % build measurement model Jcobian matrix
        qw = qt(1); qx = qt(2); qy = qt(3); qz = qt(4);
        Ht = 2.0*[gy*qz-gz*qy   gy*qy+gz*qz          -2*gx*qy+gy*qx-gz*qw  -2*gx*qz+gy*qw+gz*qx
                  -gx*qz+gz*qx  gx*qy-2*gy*qx+gz*qw  gx*qx+gz*qz           -gx*qw-2*gy*qz+gz*qy
                  gx*qy-gy*qx   gx*qz-gy*qw-2*gz*qx  gx*qw+gy*qz-2*gz*qy   gx*qx+gy*qy
                  ry*qz-rz*qy   ry*qy+rz*qz          -2*rx*qy+ry*qx-rz*qw  -2*rx*qz+ry*qw+rz*qx
                  -rx*qz+rz*qx  rx*qy-2*ry*qx+rz*qw  rx*qx+rz*qz           -rx*qw-2*ry*qz+rz*qy
                  rx*qy-ry*qx   rx*qz-ry*qw-2*rz*qx  rx*qw+ry*qz-2*rz*qy   rx*qx+ry*qy         ];
        St = Ht*Pt*Ht' + R;
        
        % Step 5) Compute Kalman gain
        Kt = Pt*Ht'*inv(St);
        
        % Step 6) Correct state quaternion and its covariance
        qt = qt + Kt*vt;
        qt = qt/norm(qt); % normalize the quaternion
        Pt = (eye(4) - Kt*Ht)*Pt;
        
        % Store data
        Pt = Pt_;
        q(k,:) = qt'; % quaternion as row (1x4)
    end
    
    

    

    
end