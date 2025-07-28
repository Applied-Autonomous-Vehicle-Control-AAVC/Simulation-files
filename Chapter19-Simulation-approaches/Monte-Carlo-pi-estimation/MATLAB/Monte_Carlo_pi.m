clear all, close all, clc

r = 1;
N = 1000; % number of dots
x = r * (2 * rand(1,N) - 1);
y = r * (2 * rand(1,N) - 1);

n_inside = 0;
n_outside = 0;

for i = 1:length(x)
    D(i) = sqrt(x(i)^2 + y(i)^2);
    if D(i) < r
        n_inside = n_inside + 1;
    else
        n_outside = n_outside + 1;
    end
end

pi_estimate = 4 * (n_inside / N);

% Plot
figure
hold on
axis equal
box on

% Plot square (no DisplayName here)
rectangle('Position', [-1 -1 2 2], 'EdgeColor', [0.5 0.5 0.5], ...
          'LineStyle', '--', 'LineWidth', 1.2);
h_square = plot(nan, nan, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

% Plot unit circle
theta = linspace(0, 2*pi, 500);
h_circle = plot(r*cos(theta), r*sin(theta), 'k-', 'LineWidth', 1.5);

% Separate inside/outside dots
D = sqrt(x.^2 + y.^2);
h_in = plot(x(D < r), y(D < r), 'b+', 'DisplayName', 'Inside');
h_out = plot(x(D >= r), y(D >= r), 'rx', 'DisplayName', 'Outside');

% Axis settings
xlim([-1.5 1.5])
ylim([-1.5 1.5])
xticks(-1.5:0.5:1.5)
yticks(-1.5:0.5:1.5)

% Labels and title
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14)
title(sprintf('$\\pi \\approx %.4f$, $n = %d$', pi_estimate, N), ...
      'Interpreter', 'latex', 'FontSize', 14)

% Legend
legend([h_circle, h_square, h_in, h_out], ...
       {'Circle', 'Square', 'Inside', 'Outside'}, ...
       'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 12)