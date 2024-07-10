function mag_bias = mag_cal(mag)
    % Calibrate magnetometer by fitting its data to a sphere. The
    % coordinates of the center of the sphere are the magnetometer bias
    % (or hard iron error).
    %
    % Inputs: mag
    %     mag: Nx3 tri-axial magnetometer readings (mx, my, mz) in gauss
    %
    % Outputs: mag_bias
    %     mag_bias: 1x3 tri-axial magnetometer bias (bx, by, bz) in gauss
    
    % measured magnetic field
    mx = mag(:,1);
    my = mag(:,2);
    mz = mag(:,3);

    % number of samples
    N = size(mx, 1);
    
    % Spherical fitting error model calibration --> h = m - b
    % h : true local magnetic field
    % m : measured magnetic field
    % A : soft iron error matrix (transformation)
    % b : hard iron error vector (bias)

    % Spherical parametric equation:
    % x^2 + y^2 + z^2 + a*x + b*y + c*z + d = H^2

    % Least Squares regresion to spherical parametric equation:
    % H^2 = h'*h = (m-b)'*(m-b) = m'*m - 2*b'*m + b'*b
    % Y = m'*m = mx^2 + my^2 + mz^2 ; X = [mx my mz 1]
    % a = [2*bx 2*by 2*bz H^2-bx^2-by^2-bz^2]
    % a = inv(X'*X)*X'*Y

    Y = mx.^2 + my.^2 + mz.^2;
    X = zeros(N,4);
    X(:,1) = mx;
    X(:,2) = my;
    X(:,3) = mz;
    X(:,4) = 1;
    a = inv(X'*X)*X'*Y;
    
    bx = 0.5*a(1);
    by = 0.5*a(2);
    bz = 0.5*a(3);
    mag_bias = [bx by bz];
    
end