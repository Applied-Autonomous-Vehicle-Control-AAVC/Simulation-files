function [x, y, z, Vx, Vy, Vz] = Rbn2(x, y, z, Vx, Vy, Vz, ax, ay, az, wx, wy, wz, phi, theta, psi, dt)
    % Define the Rotation Matrix C_b_n (Body to NED)
    C_b_n = [cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
             sin(psi)*cos(theta), cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
             -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    
    % Transform accelerations to NED frame
    accel_ned = C_b_n * [ax; ay; az];
    
    % Compute velocity updates due to angular rates
    ang_vel_ned = C_b_n * [wx; wy; wz];
    
    % Compute the Coriolis-like effects
    Vx = Vx + (accel_ned(1) + ang_vel_ned(2)*Vz - ang_vel_ned(3)*Vy) * dt;
    Vy = Vy + (accel_ned(2) + ang_vel_ned(3)*Vx - ang_vel_ned(1)*Vz) * dt;
    Vz = Vz + (accel_ned(3) + ang_vel_ned(1)*Vy - ang_vel_ned(2)*Vx) * dt;
    
    % Update position in NED frame (P = P + V * dt)
    x = x + Vx * dt;
    y = y + Vy * dt;
    z = z + Vz * dt;
end
