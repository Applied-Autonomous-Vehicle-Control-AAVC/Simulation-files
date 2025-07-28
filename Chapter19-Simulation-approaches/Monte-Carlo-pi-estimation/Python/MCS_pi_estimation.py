import numpy as np
import matplotlib.pyplot as plt

# Enable LaTeX rendering (requires external LaTeX installation)
# Comment of you don't want lbels Latex
plt.rcParams['text.usetex'] = True

# Parameters
r = 1
N = 10000  # number of dots
x = r * (2 * np.random.rand(N) - 1)
y = r * (2 * np.random.rand(N) - 1)

# Monte Carlo Estimation
n_inside = 0
n_outside = 0
D = np.zeros(N)

for i in range(N):
    D[i] = np.sqrt(x[i]**2 + y[i]**2)
    if D[i] < r:
        n_inside += 1
    else:
        n_outside += 1

pi_estimate = 4 * (n_inside / N)

# Plot
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.grid(False)
ax.set_box_aspect(1)

# Plot square
square = plt.Rectangle((-1, -1), 2, 2, edgecolor=(0.5, 0.5, 0.5),
                       linestyle='--', linewidth=1.2, facecolor='none')
ax.add_patch(square)
h_square, = ax.plot([], [], '--', color=(0.5, 0.5, 0.5), linewidth=1.2)

# Plot unit circle
theta = np.linspace(0, 2*np.pi, 500)
h_circle, = ax.plot(r * np.cos(theta), r * np.sin(theta), 'k-', linewidth=1.5)

# Separate inside/outside points
D_full = np.sqrt(x**2 + y**2)
inside_mask = D_full < r
outside_mask = ~inside_mask

h_in = ax.plot(x[inside_mask], y[inside_mask], 'b+', label='Inside')[0]
h_out = ax.plot(x[outside_mask], y[outside_mask], 'rx', label='Outside')[0]

# Axis settings
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
ax.set_yticks(np.arange(-1.5, 1.6, 0.5))

# Labels and Title
ax.set_xlabel(r'$x$', fontsize=14)
ax.set_ylabel(r'$y$', fontsize=14)
ax.set_title(r'$\pi \approx %.4f$, $n = %d$' % (pi_estimate, N),
             fontsize=14)

# Legend
ax.legend([h_circle, h_square, h_in, h_out],
          ['Circle', 'Square', 'Inside', 'Outside'],
          loc='upper right',
          bbox_to_anchor=(1.2, 1.05),  # adjust as needed
          fontsize=12,
          framealpha=1)

#plt.savefig('MCS_pi_estimation.eps', format='eps', bbox_inches='tight')
plt.savefig('MCS_pi_estimation.pdf', bbox_inches='tight')

plt.show()
