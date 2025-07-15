% common font sizes
f_size_lab    = 14;
f_size_ticks  = 12;
f_size_legend = 10;

%% Figure 1: AUV trajectories with start/goal/obstacle markers
figure(1)
hold on

% trajectories
plot(AUV1.signals.values(:,1), AUV1.signals.values(:,2), 'k',   'LineWidth',2)
plot(AUV2.signals.values(:,1), AUV2.signals.values(:,2), '--r', 'LineWidth',2)
plot(AUV3.signals.values(:,1), AUV3.signals.values(:,2), '-.g', 'LineWidth',2)
plot(AUV4.signals.values(:,1), AUV4.signals.values(:,2), ':b',  'LineWidth',2)

% initial positions as empty circles
plot(xAUV_ini(1), yAUV_ini(1), 'o', 'MarkerSize',11, 'MarkerEdgeColor','k', 'MarkerFaceColor','none')
plot(xAUV_ini(2), yAUV_ini(2), 'o', 'MarkerSize',11, 'MarkerEdgeColor','r', 'MarkerFaceColor','none')
plot(xAUV_ini(3), yAUV_ini(3), 'o', 'MarkerSize',11, 'MarkerEdgeColor','g', 'MarkerFaceColor','none')
plot(xAUV_ini(4), yAUV_ini(4), 'o', 'MarkerSize',11, 'MarkerEdgeColor','b', 'MarkerFaceColor','none')

% obstacle
plot(Xob, Yob, 'o', 'MarkerSize',15, 'MarkerFaceColor','k', 'MarkerEdgeColor','k')

% goal positions
plot(targetX(1), targetY(1), 'kp', 'MarkerSize',15)
plot(targetX(2), targetY(2), 'rp', 'MarkerSize',15)
plot(targetX(3), targetY(3), 'gp', 'MarkerSize',15)
plot(targetX(4), targetY(4), 'bp', 'MarkerSize',15)

% axes labels
xlabel('$x (m)$', 'FontSize',f_size_lab, 'Interpreter','latex')
ylabel('$y (m)$', 'FontSize',f_size_lab, 'Interpreter','latex')

% legend placeholders
h = gobjects(7,1);
h(1) = plot(NaN,NaN,'k','LineWidth',2);
h(2) = plot(NaN,NaN,'--r','LineWidth',2);
h(3) = plot(NaN,NaN,'-.g','LineWidth',2);
h(4) = plot(NaN,NaN,':b','LineWidth',2);
h(5) = plot(NaN,NaN,'ob','MarkerSize',11,'MarkerFaceColor','b', 'MarkerFaceColor','none');
h(6) = plot(NaN,NaN,'pb','MarkerSize',15);
h(7) = plot(NaN,NaN,'ok','MarkerSize',15,'MarkerFaceColor','k');

legend(h, {
    '$AUV_1$','$AUV_2$','$AUV_3$','$AUV_4$', ...
    '$Start$','$Goal$','$Obstacle$'}, ...
    'NumColumns',2, ...
    'FontSize',12, ...
    'Interpreter','latex', ...
    'Location','best')

% formatting
set(gca, 'FontSize',f_size_ticks, 'TickLabelInterpreter','latex')
xlim([-20 140])
ylim([-20 140])
hold off


%% Figure 2: inter‐vehicle distances
% compute Rs already done above
% Fetching data
x1 = xys(1,:); y1 = xys(2,:);
x2 = xys(3,:); y2 = xys(4,:);
x3 = xys(5,:); y3 = xys(6,:);
x4 = xys(7,:); y4 = xys(8,:);

R12 = sqrt((x1-x2).^2+(y1-y2).^2);
R13 = sqrt((x1-x3).^2+(y1-y3).^2);
R14 = sqrt((x1-x4).^2+(y1-y4).^2);
R23 = sqrt((x2-x3).^2+(y2-y3).^2);
R24 = sqrt((x2-x4).^2+(y2-y4).^2);
R34 = sqrt((x3-x4).^2+(y3-y4).^2);

figure(2)
plot(t, R12, '-k', t, R13, '--r', t, R14, '-.g', t, R23, ':b', t, R24, '-.m', t, R34, '--c', 'LineWidth',1.5)

xlabel('Time (s)', 'FontSize', f_size_lab, 'Interpreter', 'latex')
ylabel('Distances $\rho_{ij}$ (m)', 'FontSize', f_size_lab, 'Interpreter', 'latex')

legend({
    '$\rho_{12}$', '$\rho_{13}$', '$\rho_{14}$', ...
    '$\rho_{23}$', '$\rho_{24}$', '$\rho_{34}$'}, ...
    'FontSize',     f_size_legend, ...
    'Interpreter',  'latex', ...
    'Location',     'best', ...
    'Orientation',  'horizontal', ...
    'NumColumns',   6)

set(gca, 'FontSize',f_size_ticks, 'TickLabelInterpreter','latex')
xlim([0, t(end)])
ylim([0, 45])


%% Figure 3: control efforts u
figure(3)
plot(t, u1, '-k', t, u2, '--r', t, u3, '-.g', t, u4, ':b', 'LineWidth',1.5)

xlabel('Time (s)', 'FontSize',f_size_lab, 'Interpreter','latex')
ylabel('$u$ for AUVs (m/s)', 'FontSize',f_size_lab, 'Interpreter','latex')

legend({'$AUV_1$','$AUV_2$','$AUV_3$','$AUV_4$'}, ...
       'FontSize',f_size_legend, ...
       'Interpreter','latex', ...
       'Location','best')

set(gca, 'FontSize',f_size_ticks, 'TickLabelInterpreter','latex')
xlim([0, t(end)])
ylim([0 2.0])

%% Figure 4: rudder angles delta_r (in deg)
figure(4)
plot(t, delta_r1*180/pi, '-k', t, delta_r2*180/pi, '--r', t, delta_r3*180/pi, '-.g', t, delta_r4*180/pi, ':b', 'LineWidth',1.5)

xlabel('Time (s)', 'FontSize',f_size_lab, 'Interpreter','latex')
ylabel('$\delta_{r}\ \mathrm{for\ AUVs}\ \left(^\circ\right)$', ...
       'FontSize',f_size_lab, ...
       'Interpreter','latex')

legend({
    '$AUV_1$','$AUV_2$','$AUV_3$','$AUV_4$'}, ...
    'FontSize',f_size_legend, ...
    'Interpreter','latex', ...
    'Location','best')

set(gca, 'FontSize',f_size_ticks, 'TickLabelInterpreter','latex')
xlim([0, 140])
ylim([-40 40])
