function [lat, lon, h] = VNED2LLA(x, y, z, Vx, Vy, Vz, lat_prev, lon_prev, h_prev, dt)
    % WGS84 ellipsoid constants
    a = 6378137.0; % Semi-major axis (m)
    e = 0.081819190842622; % Eccentricity

    % Compute radius of curvature in the prime vertical
    R_N = a / sqrt(1 - e^2 * sin(lat_prev)^2); % Meridional radius
    R_E = R_N / (1 - e^2); % Transverse radius

    % Update latitude (radians)
    lat = lat_prev + (dt * Vx) / R_N;

    % Update longitude (radians)
    lon = lon_prev + (dt * Vy) / (R_E * cos(lat_prev));

    % Update altitude (meters)
    h = h_prev - dt * Vz;
end