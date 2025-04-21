clear all, close all, clc

Sim_time = 70; %s
dt = 0.01; % integration time

% Initialisation
time = 0;
i = 2;
t = 0.01;

% Given coordinates (in degrees)
lat = 51.4927*pi/180; % Latitude
lon = -0.0098*pi/180; % Longitude
h = 100; % Altitude (meters)

% Initial conditions
% Positions in NED
x = 0;
y = 0;
z = 0;

% Compute correct initial ECEF coordinates
a = 6378137.0; % WGS84 semi-major axis
e = 0.081819190842622; % WGS84 eccentricity
N = a / sqrt(1 - e^2 * sin(lat)^2);

x_ecef = (N + h) * cos(lat) * cos(lon); x1=x_ecef;
y_ecef = (N + h) * cos(lat) * sin(lon); y1=y_ecef;
z_ecef = (N * (1 - e^2) + h) * sin(lat); z1=z_ecef;

wgs84 = wgs84Ellipsoid('meter');

% Velocities
Vx = 0;
Vy = 0;
Vz = 0;
% Euler angles
phi = 0*pi/180;
theta = 0*pi/180;
psi = -0*pi/180;

fy = 0;
fx = 0;
fz = -9.81;

% index
last_update_i = i;

while t <= Sim_time
    time(i) = t;

    % Vehicle experienced motion in body frame
    if t < 10 - dt
        ax(i) = 2.5;
        ay(i) = 0;
        az(i) = 0;
    else
        ax(i) = 0;
        ay(i) = 0;
        az(i) = 0;
    end
    
    wx(i) = 0; % Placeholder for gyro measurements
    wy(i) = 0;
    wz(i) = 0;

    if (t>10 & t<15)
        wx(i) = 0.0*pi/180;
    end

    if (t>35 & t<45)
        wz(i) = 9*pi/180;
    end

    % if (t>50 & t<60)
    %     wz(i) = 9*pi/180;
    % end

    % if (t>100 & t<110)
    %     wz(i) = 4.5*pi/180;
    % end


    % Update Euler angles (Orientation)
    [phi(i), theta(i), psi(i)] = Rbn1(phi(i-1), theta(i-1), psi(i-1), wx(i), wy(i), wz(i), dt);
    
    % Update positions and velocities (Translation)
    [x(i), y(i), z(i), Vx(i), Vy(i), Vz(i)] = Rbn2(x(i-1), y(i-1), z(i-1), Vx(i-1), Vy(i-1), Vz(i-1), ax(i), ay(i), az(i), wx(i), wy(i), wz(i), phi(i), theta(i), psi(i), dt);

    % Update data from NED to LLA (GPS positions)
    % This is the main approach
    [lat(i), lon(i), h(i)] = VNED2LLA(x(i), y(i),z(i),Vx(i), Vy(i),Vz(i), lat(i-1), lon(i-1), h(i-1),dt);
    
    % Accelerometer modelling
    [fx(i), fy(i), fz(i), gx(i), gy(i), gz(i)] = Gravity_Modelling(ax(i), ay(i), az(i), phi(i), theta(i), psi(i), lat(i));
    
    % Inclinometer calculations
    g0 = 9.81;
    phi_m(i) = atan2(-fy(i),-fz(i));
    theta_m(i) = asin(fx(i) / g0);


    % Time accumulation
    t = t + dt;
    i = i + 1;
end

Plotting_Nav

figure;
geoplot(lat*180/pi, lon*180/pi, 'r', 'LineWidth', 2); 
geobasemap('satellite'); 
hold on;

geoscatter(lat(1) * 180/pi, lon(1) * 180/pi, 100, 'w', 'v', 'filled'); % Green start pin
geoscatter(lat(end) * 180/pi, lon(end) * 180/pi, 100, 'g', 'v', 'filled'); % Blue end pin

% Add text labels near the points
text(lat(1) * 180/pi, lon(1) * 180/pi, sprintf('  Point A:\n  Lat = %.4f°\n  Lon = %.4f°', lat(1) * 180/pi, lon(1) * 180/pi), ...
    'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');

text(lat(end) * 180/pi, lon(end) * 180/pi, sprintf('  Point B:\n  Lat = %.4f°\n  Lon = %.4f°', lat(end) * 180/pi, lon(end) * 180/pi), ...
    'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

title('Geographic Path Plot');


% %%% Open webmap with Esri World Imagery
% wm = webmap('World Imagery'); % Use Esri’s high-resolution satellite imagery
% wmline(lat*180/pi, lon*180/pi, 'Color', 'r', 'LineWidth', 4); % Plot the path
