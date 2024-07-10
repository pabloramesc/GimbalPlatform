clear; clc; close all;

load('datasets/platform/test.mat');

% measured magnetic field
mx = data(:,8);
my = data(:,9);
mz = data(:,10);
m = [-mx; my; -mz];

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

bx = 0.5*a(1)
by = 0.5*a(2)
bz = 0.5*a(3)
H = sqrt(a(4) + bx^2 + by^2 + bz^2)

hx = mx - bx;
hy = my - by;
hz = mz - bz;

figure(1)
title ('Magnetometer XY calibration data')
xlabel('mx')
ylabel('my')
hold on
plot(mx, my, '.')
plot(hx, hy, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(2)
title ('magnetometer XZ data')
xlabel('mx')
ylabel('mz')
hold on
plot(mx, mz, '.')
plot(hx, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(3)
title ('magnetometer YZ data')
xlabel('my')
ylabel('mz')
hold on
plot(my, mz, '.')
plot(hy, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(4)
title ('Magnetometer 3D calibration data')
hold on
plot3(mx, my, mz, '.')
plot3(hx, hy, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off