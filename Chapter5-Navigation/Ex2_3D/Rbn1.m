function [phi, theta, psi] = Rbn1(phi, theta, psi, wx, wy, wz, dt)
    % Rbn1: Updates Euler angles from angular velocities (gyroscopes)

    % Rotation matrix based on Euler angle kinematics
    R_euler = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi), -sin(phi);
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % Angular velocity components (body frame)
    omega_body = [wx; wy; wz];
    
    % Calculate Euler angle rates by multiplying rotation matrix with omega_body
    angle_rates = R_euler * omega_body;
    
    % Extract Euler angle rates (phi_dot, theta_dot, psi_dot)
    phi_dot = angle_rates(1);
    theta_dot = angle_rates(2);
    psi_dot = angle_rates(3);
    
    % Integration to get Euler angles
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi = psi + psi_dot * dt;
end