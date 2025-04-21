function [fx, fy, fz, gx, gy, gz] = gravity_modelling(ax, ay, az, phi, theta, psi, lat)
    % More precise gravity calculation using latitude, longitude, and altitude

    % WGS84 parameters
    a = 6378137.0; % Semi-major axis (m)
    e = 0.081819190842622; % Eccentricity
    gamma_e = 9.7803253359; % Equatorial gravity (m/s²)
    gamma_p = 9.8321849378; % Polar gravity (m/s²)

    % Compute gravity at given latitude (Somigliana's formula)
    sin2_lat = sin(lat)^2;
    g = (gamma_e * (1 + 0.00193185265241 * sin2_lat)) / sqrt(1 - e^2 * sin2_lat);

    % Gravity vector in NED frame (pointing down)
    gx = 0; 
    gy = 0; 
    gz = -g;  % Gravity vector pointing downwards

    % Compute rotation matrix from NED to Body frame
    R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
         sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
        -sin(theta),         cos(theta)*sin(phi),                           cos(theta)*cos(phi)];

    gravity_body = R' * [gx; gy; gz];  % Rotate gravity vector from NED to Body frame

    % Calculate forces by subtracting gravity
    fx = ax + gravity_body(1); 
    fy = ay + gravity_body(2); 
    fz = az + gravity_body(3);  % Subtract the gravity vector from accelerometer readings
end