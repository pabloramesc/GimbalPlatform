clear; clc; close all;

% Import data to analyze
load('datasets/platform/test.mat');

t = data(:,1); % time elapsed (seconds)
tx = data(:,2); % AS5600 measured angles (degrees)
ty = data(:,3);
tz = data(:,4);
ax = data(:,5); % BLE33 accelerometer readings (Gs)
ay = data(:,6);
az = data(:,7);
gx = data(:,8); % BLE33 gyroscope readings (deg/s)
gy = data(:,9);
gz = data(:,10);
mx = data(:,11); % BLE33 magnetometer readings (uT)
my = data(:,12);
mz = data(:,13);
dt = data(:,14)/1000.0; % BLE33 internal clock dt (ms)

omega = az; % variable to analyze
ts = mode(dt); % the constant sampling period as mode
max_clusters = 100; % max clusters number to use (recommended 100)

% Compute the Allan Variance
theta = cumsum(omega, 1)*ts;
L = size(theta, 1); % number of samples
m = 2.^floor(log2(L/2)); % size of the biggest cluster
% build log spaced clusters
clusters = unique(ceil(logspace(log10(1), log10(m), max_clusters).')); 
tau = clusters*ts;
avar = zeros(numel(clusters), 1);
for k = 1:numel(clusters)
    mk = clusters(k);
    avar(k,:) = sum( ...
        (theta(1+2*mk:L) - 2*theta(1+mk:L-mk) + theta(1:L-2*mk)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2+clusters));
adev = sqrt(avar); % get the Allan Standard Deviation


% Get the N allan factor
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, k] = min(abs(dlogadev - slope));
b = logadev(k) - slope*logtau(k);
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Get the K allan factor
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, k] = min(abs(dlogadev - slope));
b = logadev(k) - slope*logtau(k);
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

%Get the B allan factor
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, k] = min(abs(dlogadev - slope));
b = logadev(k) - slope*logtau(k);
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(k);
lineB = B * scfB * ones(size(tau));

% Plot the results
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', tauParams, params, 'o')
title('Allan Standard Deviation with Noise Parameters')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N', '\sigma_K', '\sigma_B')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal