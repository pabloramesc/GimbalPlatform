function q = gyro_integration(gyr, q0, Dt)
    % Estimate attitutde integrating the angular rates measured by the
    % gyroscope. To avoid locking gymbal problem the integration will be
    % perfomed with quaternions.
    %
    % Inputs: gyr, q0, Dt
    %     gyr: Nx3 tri-axial gyroscope readings (wx, wy, wz) in rad/s
    %     q0: 1x4 initial attitude as quaternion
    %     Dt: N sampling periods array in seconds
    %
    % Outputs: q
    %     q: Nx4 normalized quaternions array with the attitude

    N = size(gyr, 1); % get the number of samples
    q = zeros(N,4);
    q(1,:) = q0;
    for k = 2:N
        dt = Dt(k); % current sampling period
        wx = gyr(k,1); wy = gyr(k,2); wz = gyr(k,3);
        % compute Omega matrix
        Omega = [ 0.0  -wx  -wy  -wz
                  +wx  0.0  +wz  -wy
                  +wy  -wz  0.0  +wx
                  +wz  +wy  -wx  0.0 ];
        qt_prev = q(k-1,:)'; % get las quaternion as column (4x1)
        qt = (0.5*Omega*dt + eye(4))*qt_prev;
        qt = qt/norm(qt); % normalize the quaternion
        q(k,:) = qt.'; % store the computed quaternion as row (1x4)
    end
end