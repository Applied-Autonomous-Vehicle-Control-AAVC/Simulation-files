"""
Monte Carlo Simulation and Statistical Analysis of an AUV
Forked from: https://github.com/Psarhadi/AUV-Autonomous-Underwater-Vehicle-Six-DOF-Simulation
The code performs Monte Carlo simulations on a detailed Autonomous Underwater Vehicle model 
Code by: Pouria Sarhadi
"""

import numpy as np
import matplotlib.pyplot as plt

try:
    import AUV_model
except:
    raise

import time as tm

# For Latex (MikTex) labels, if not installed or needed comment
# plt.rcParams['text.usetex'] = True

dt = 0.1  # solver and sampling time [s]
Tf = 80.0  # simulation time [s]

No_Run = 100 # Monte Carlo Simulation number of runs

class Guidance():
    def Longitudinal(time):
        z_c = 30.0
        theta_c = 0.0
        fin_val = 50.0
        time_cons = 40.0
        if time > 10.0 and time <= 10.0 + time_cons:
            z_c = 30.0 + (time - 10.0) * ((fin_val - 30.0) / time_cons)
        elif time > 10.0 + time_cons:
            z_c = fin_val
        return z_c, theta_c

    def Lateral(time):
        psi_c = 0.0
        if t >= 30.0:
            psi_c = np.deg2rad(15.0)
        if t >= 60.0:
            psi_c = np.deg2rad(0.0)
        return psi_c

class Control():
    def Longitudinal(theta_com, z_com, z, theta, q, e_i_z1):
        Kpz = -0.5
        Kiz = -0.00
        Kth = 4.0
        Kq = 2.0
        e_z = z_com - z
        e_i_z = dt * e_z + e_i_z1
        u_z = Kpz * e_z + Kiz * e_i_z
        del_e = -(u_z - Kth * theta - Kq * q)
        return del_e, e_i_z

    def Lateral(psi_com, psi, r, u_i_psi1):
        Kp = 4.0
        Ki = 0.1
        Kr = 3.0
        e_psi = psi_com - psi
        u_i_psi = dt * e_psi + u_i_psi1
        del_r = -(Kp * e_psi + Ki * u_i_psi - Kr * r)
        return del_r, u_i_psi

def Actuator_modelling(delta):
    max_deflection = np.deg2rad(20.0)
    delta_ac = np.clip(delta, -max_deflection, max_deflection)
    return delta_ac

def Runge_Kutta(dx, state, inputs, dt):
    f1 = dx(state, inputs)
    x1 = state + (dt / 2.0) * f1
    f2 = dx(x1, inputs)
    x2 = state + (dt / 2.0) * f2
    f3 = dx(x2, inputs)
    x3 = state + dt * f3
    f4 = dx(x3, inputs)
    return state + (dt / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4)

def main():
    print("Start " + __file__)
    global t

    # Containers for multi-run storage
    time_MCS, z_MCS, z_com_MCS, delta_e_ac_MCS = [], [], [], []
    psi_MCS, psi_com_MCS, delta_r_ac_MCS, AUX_MCS = [], [], [], []

    # Also save all other variables
    u_MCS, v_MCS, w_MCS = [], [], []
    p_MCS, q_MCS, r_MCS = [], [], []
    x_MCS, y_MCS = [], []
    phi_MCS, theta_MCS = [], []
    delta_e_MCS, delta_r_MCS = [], []
    theta_com_MCS, psi_com_raw_MCS = [], []
    zf_runs, psif_runs = [], []  # Final values per run
    zf_run_mean_list, zf_run_std_list = [], []
    psif_run_mean_list, psif_run_std_list = [], []
    start_time = tm.time()
    for run_id in range(No_Run):
        t = 0.0
        e_i_z1 = 0.0
        u_i_psi1 = 0.0

        time = [0.0]
        delta_e, delta_r = [0.0], [0.0]
        delta_e_ac, delta_r_ac = [0.0], [0.0]
        AUX = [0.0]

        X0 = [
            1.8 + 0.2 * np.random.randn(),                     # u
            0.0 + 0.2 * np.random.randn(),                     # v
            0.0 + 0.2 * np.random.randn(),                     # w
            0.0 + np.deg2rad(5.0) * np.random.randn(),         # p
            0.0 + np.deg2rad(5.0) * np.random.randn(),         # q
            0.0 + np.deg2rad(5.0) * np.random.randn(),         # r
            0.0,                                               # x
            0.0,                                               # y
            30.0 + 2.0 * np.random.randn(),                    # z
            0.0 + np.deg2rad(2.0) * np.random.randn(),         # phi
            0.0 + np.deg2rad(2.0) * np.random.randn(),         # theta
            0.0 + np.deg2rad(25.0) * np.random.randn()         # psi
        ]
        X = X0
        
        AUV_model.define_randomize_parameters()
        print("X_T =", AUV_model.X_T)
        #print("W =", AUV_model.W)

        # Initialize single-run logs
        u, v, w = [X0[0]], [X0[1]], [X0[2]]
        p, q, r = [X0[3]], [X0[4]], [X0[5]]
        x, y, z = [X0[6]], [X0[7]], [X0[8]]
        phi, theta, psi = [X0[9]], [X0[10]], [X0[11]]

        theta_com, z_com, psi_com = [0.0], [30.0], [0.0]

        while Tf >= t:
            t += dt
            z_c, theta_c = Guidance.Longitudinal(t)
            psi_c = Guidance.Lateral(t)

            d_e, e_i_z1 = Control.Longitudinal(theta_c, z_c, X[8], X[10], X[4], e_i_z1)
            d_r, u_i_psi1 = Control.Lateral(psi_c, X[11], X[5], u_i_psi1)

            d_e_ac = Actuator_modelling(d_e) + np.deg2rad(0.2)*np.random.randn()
            d_r_ac = Actuator_modelling(d_r)

            inputs = [d_e_ac, d_r_ac]
            X = Runge_Kutta(AUV_model.AUV, X0, inputs, dt)
            X0 = X

            # Save single-run values
            time.append(t)
            u.append(X[0]); v.append(X[1]); w.append(X[2])
            p.append(X[3]); q.append(X[4]); r.append(X[5])
            x.append(X[6]); y.append(X[7]); z.append(X[8])
            phi.append(X[9]); theta.append(X[10]); psi.append(X[11])

            theta_com.append(theta_c); z_com.append(z_c); psi_com.append(psi_c)
            delta_e.append(d_e); delta_r.append(d_r)
            delta_e_ac.append(d_e_ac); delta_r_ac.append(d_r_ac)
            AUX.append(np.rad2deg(e_i_z1))

        # Store into MCS containers
        time_MCS.append(np.array(time))
        z_MCS.append(np.array(z))
        z_com_MCS.append(np.array(z_com))
        delta_e_ac_MCS.append(np.rad2deg(delta_e_ac))

        psi_MCS.append(np.array(psi))
        psi_com_MCS.append(np.array(psi_com))
        delta_r_ac_MCS.append(np.array(delta_r_ac))
        AUX_MCS.append(np.array(AUX))
        
        # Here mean std for z_f and psi_f, call zf_run_mean std and same for psif 
        # Accumulate final values
        zf_runs = zf_runs + [z[-1]] if run_id else [z[-1]]
        psif_runs = psif_runs + [psi[-1]] if run_id else [psi[-1]]

        # Compute stats
        zf_arr = np.array(zf_runs)
        psif_arr = np.rad2deg(np.array(psif_runs))

        # Append evolving statistics
        zf_run_mean_list.append(np.mean(zf_arr))
        zf_run_std_list.append(np.std(zf_arr))
        psif_run_mean_list.append(np.mean(psif_arr))
        psif_run_std_list.append(np.std(psif_arr))

        u_MCS.append(np.array(u)); v_MCS.append(np.array(v)); w_MCS.append(np.array(w))
        p_MCS.append(np.array(p)); q_MCS.append(np.array(q)); r_MCS.append(np.array(r))
        x_MCS.append(np.array(x)); y_MCS.append(np.array(y))
        phi_MCS.append(np.array(phi)); theta_MCS.append(np.array(theta))
        delta_e_MCS.append(np.array(delta_e)); delta_r_MCS.append(np.array(delta_r))
        theta_com_MCS.append(np.array(theta_com)); psi_com_raw_MCS.append(np.array(psi_com))
    
    end_time = tm.time()
    print("Elapsed time:", f"{end_time - start_time:.2f}", "sec")

    # Convert to arrays once
    time_MCS = np.array(time_MCS)
    z_MCS = np.array(z_MCS)
    z_com_MCS = np.array(z_com_MCS)
    delta_e_ac_MCS = np.array(delta_e_ac_MCS)

    psi_MCS = np.rad2deg(np.array(psi_MCS))         # radians to degrees
    psi_com_MCS = np.rad2deg(np.array(psi_com_MCS)) # radians to degrees
    delta_r_ac_MCS = np.rad2deg(np.array(delta_r_ac_MCS))
    AUX_MCS = np.array(AUX_MCS)

    # Extract time vector from first run (assumes all same length)
    time = time_MCS[0]

    # Compute means and std devs for z and elevator
    z_mean = -np.mean(z_MCS, axis=0)
    z_std = -np.std(z_MCS, axis=0)
    z_com_mean = -np.mean(z_com_MCS, axis=0)

    delta_e_mean = np.mean(delta_e_ac_MCS, axis=0)
    delta_e_std = np.std(delta_e_ac_MCS, axis=0)
    
    # Plot Depth z and elevator deflection
    fig, axs = plt.subplots(2, 1)

    axs[0].plot(time, z_com_mean, 'r--', label=r'$z_{com}$')
    axs[0].plot(time, z_mean, 'b-', label=r'$\bar{z}$')
    axs[0].fill_between(time, z_mean - z_std, z_mean + z_std, color='lightblue', label=r'$\pm \sigma_z$')
    axs[0].set_ylabel(r'$z \; (m)$', fontsize=14)
    axs[0].legend()
    axs[0].set_title('Longitudinal control signals over Monte Carlo runs', fontsize=14)

    axs[1].plot(time, delta_e_mean, 'b-', label=r'$\bar{\delta}_{eac}$')
    axs[1].fill_between(time, delta_e_mean - delta_e_std, delta_e_mean + delta_e_std, color='lightblue', label=r'$\pm \sigma_{\delta_{eac}}$')
    axs[1].set_xlabel(r'$Time \; (s)$', fontsize=14)
    axs[1].set_ylabel(r'$\delta_{eac} \; (^\circ)$', fontsize=14)
    axs[1].legend()

    plt.tight_layout()
    plt.savefig("MCS_z.pdf", format='pdf', dpi=300)

    plt.show()

    # psi plot
    psi_mean = np.mean(psi_MCS, axis=0)
    psi_std = np.std(psi_MCS, axis=0)
    psi_com_mean = np.mean(psi_com_MCS, axis=0)

    delta_r_mean = np.mean(delta_r_ac_MCS, axis=0)
    delta_r_std = np.std(delta_r_ac_MCS, axis=0)

    fig, axs = plt.subplots(2, 1)

    axs[0].plot(time, psi_com_mean, 'r--', label=r'$\psi_{com}$')
    axs[0].plot(time, psi_mean, 'b-', label=r'$\bar{\psi}$')
    axs[0].fill_between(time, psi_mean - psi_std, psi_mean + psi_std, color='lightblue', label=r'$\pm \sigma_\psi$')
    axs[0].set_ylabel(r'$\psi \; (^\circ)$', fontsize=14)
    axs[0].legend(loc='lower right')
    axs[0].set_title('Lateral control over Monte Carlo runs', fontsize=14)

    axs[1].plot(time, delta_r_mean, 'b-', label=r'$\bar{\delta}_{rac}$')
    axs[1].fill_between(time, delta_r_mean - delta_r_std, delta_r_mean + delta_r_std, color='lightblue', label=r'$\pm \sigma_{\delta_{rac}}$')
    axs[1].set_xlabel(r'$Time \; (s)$', fontsize=14)
    axs[1].set_ylabel(r'$\delta_{rac} \; (^\circ)$', fontsize=14)
    axs[1].legend(loc='lower right')

    plt.tight_layout()
    plt.savefig("MCS_psi.pdf", format='pdf', dpi=300)
    plt.show()

    # CEP plot
    psi_final = psi_MCS[:, -1]
    z_final = z_MCS[:, -1]
    center_psi = np.mean(psi_final)
    center_z = np.mean(z_final)
    dist = np.sqrt((psi_final - center_psi)**2 + (z_final - center_z)**2)
    r50 = np.percentile(dist, 50)

    plt.figure(figsize=(6, 6))
    plt.plot(psi_final, z_final, 'bx', label=r'$Final \; points$')
    plt.plot(center_psi, center_z, 'r+', label=r'$Mean \; value$', markersize=8, markeredgewidth=2.0)
    plt.gca().add_patch(plt.Circle((center_psi, center_z), r50, color='g', linestyle='--',
                                linewidth=2.5, fill=False, label=r'$CEP \; 50\%$'))

    plt.xlabel(r'$\psi \; (^\circ)$', fontsize=14)
    plt.ylabel(r'$z \; (m)$', fontsize=14)
    plt.title(r'Final $\psi$ vs. $z$ with CEP', fontsize=14)
    plt.xlim(-1.0, 1.0)
    plt.ylim(49.0, 51.0)
    plt.grid()
    plt.legend()
    textstr = (r'$\bar{z}_f=%.2f\,m$' '\n' r'$\bar{\psi}_f=%.2f^\circ$' % (center_z, center_psi))
    plt.text(0.58, 49.05, textstr, fontsize=12, color='blue', 
         bbox=dict(facecolor='white', edgecolor='#d3d3d3', alpha=1, pad=4))
    plt.tight_layout()
    plt.savefig("MCS_CEP.pdf", format='pdf', dpi=300)
    plt.show()
    '''
    # u plot
    u_mean = np.mean(u_MCS, axis=0)
    u_std = np.std(u_MCS, axis=0)

    plt.plot(time, u_mean, 'b-', label=r'$\bar{u}$')
    plt.fill_between(time, u_mean - u_std, u_mean + u_std, color='lightblue', label=r'$\pm \sigma_u$')
    plt.xlabel(r'$Time \; (s)$', fontsize=14)
    plt.ylabel(r'$u \; (m/s)$', fontsize=14)
    plt.title(r'Surge velocity $u$ over Monte Carlo runs', fontsize=14)
    plt.legend()
    plt.ylim(0, 2.2)
    plt.tight_layout()
    plt.savefig("MCS_u.pdf", format='pdf', dpi=300)
    plt.show()

    # Final values per run
    runs = range(1, No_Run + 1)

    fig, axs = plt.subplots(2, 2, figsize=(10, 6))

    axs[0, 0].plot(runs, zf_run_mean_list, 'b-')
    #axs[0, 0].set_title(r'$\bar{z}_{f}$ per run', fontsize=14)
    axs[0, 0].set_xlabel(r'$Run \; number$', fontsize=14)
    axs[0, 0].set_ylabel(r'$\bar{z}_f \; (m)$', fontsize=14)

    axs[0, 1].plot(runs, zf_run_std_list, 'b-')
    #axs[0, 1].set_title(r'$\bar{\psi}_{f}$ per run', fontsize=14)
    axs[0, 1].set_xlabel(r'$Run \; number$', fontsize=14)
    axs[0, 1].set_ylabel(r'$\sigma_{z_f} \; (m)$', fontsize=14)

    axs[1, 0].plot(runs, psif_run_mean_list, 'g-')
    #axs[1, 0].set_title(r'$\bar{\psi}_{f}$ per run', fontsize=14)
    axs[1, 0].set_xlabel(r'$Run \; number$', fontsize=14)
    axs[1, 0].set_ylabel(r'$\bar{\psi}_{f} \; (^\circ)$', fontsize=14)

    axs[1, 1].plot(runs, psif_run_std_list, 'g-')
    #axs[1, 1].set_title(r'$\sigma_{\psi_{f}}$ per run', fontsize=14)
    axs[1, 1].set_xlabel(r'$Run \; number$', fontsize=14)
    axs[1, 1].set_ylabel(r'$\sigma_{\psi_{f}} \; (^\circ)$', fontsize=14)

    plt.tight_layout()
    plt.savefig("MCS_run_stats.pdf", format='pdf', dpi=300)
    plt.show()
    '''

if __name__ == '__main__':
    main()
