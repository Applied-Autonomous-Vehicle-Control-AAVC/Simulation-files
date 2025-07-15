f_size_lab    = 14;
f_size_ticks  = 12;
f_size_legend = 10;

figure(1)
hold on
plot(time, xd(1,:), 'k', 'LineWidth', 1.5)
plot(time, xd(3,:), '--r', 'LineWidth', 1.5)
plot(time, xd(5,:), '-.g', 'LineWidth', 1.5)
xlabel('$x_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('Position', 'FontSize', f_size_lab, 'Interpreter', 'latex')
legend({'$x_d$', '$y_d$', '$z_d$'}, 'FontSize', f_size_legend, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
hold off

figure(2)
plot3(xd(1,:), xd(3,:), xd(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$y_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
zlabel('$z_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

figure(3)
plot(xd(1,:), xd(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$z_d\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
axis equal
axis([-220 220 -220 220])

figure(4)
plot3(x(1,:), x(3,:), x(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$y\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
zlabel('$z\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

figure(5)
hold on
plot(time, x(1,:), 'k', 'LineWidth', 1.5)
plot(time, x(3,:), '--r', 'LineWidth', 1.5)
plot(time, x(5,:), '-.g', 'LineWidth', 1.5)
xlabel('Time (s)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('Position (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
legend({'$x$', '$y$', '$z$'}, 'FontSize', f_size_legend, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
hold off

figure(6)
hold on
plot(time, er(1,:), 'k', 'LineWidth', 1.5)
plot(time, er(3,:), '--r', 'LineWidth', 1.5)
plot(time, er(5,:), '-.g', 'LineWidth', 1.5)
xlabel('Time (s)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('Position Errors (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
legend({'$e_x$', '$e_y$', '$e_z$'}, 'FontSize', f_size_legend, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
hold off
xlim([0,2000])

figure(7)
plot(x(1,:), x(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$z\, (km)$', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

figure(8)
hold on
plot(time, u(1,:), 'k', 'LineWidth', 1.5)
plot(time, u(2,:), '--r', 'LineWidth', 1.5)
plot(time, u(3,:), '-.g', 'LineWidth', 1.5)
xlabel('Time (s)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('Control Inputs', 'FontSize', f_size_lab, 'Interpreter', 'latex')
legend({'$u_x$', '$u_y$', '$u_z$'}, 'FontSize', f_size_legend, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
hold off
xlim([0 2000])

figure(9)

subplot(2,2,1)
plot(x(1,:), x(3,:), 'k', 'LineWidth', 1.5)
xlabel('$x$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$y$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

subplot(2,2,2)
plot(x(1,:), x(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$z$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

subplot(2,2,3)
plot(x(3,:), x(5,:), 'k', 'LineWidth', 1.5)
xlabel('$y$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$z$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')

subplot(2,2,4)
plot3(x(1,:), x(3,:), x(5,:), 'k', 'LineWidth', 1.5)
xlabel('$x$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('$y$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
zlabel('$z$ (km)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
