import numpy as np
import matplotlib.pyplot as plt
import time
import random

# Plant dynamics functions
def integr(dt, inp, tm, out):
    out[0] += inp * dt / tm

def intert(dt, inp, k, tm, out):
    out[0] += (inp * k - out[0]) * dt / tm

def rldiff(dt, dinp, tdf, tm, out):
    out[0] += dinp * tdf/tm - out[0] * dt / tm
    return out[0]
# Constants
F_set = 100  
dt = 0.01    # Time step
tf = 10      # Simulation duration
step = int(tf/dt) + 1

#Kalman filter
class KalmanFilter:
    def __init__(self, dt, Kf, Kv, Tm):
        self.dt = dt
        self.A = np.array([[1, Kf*dt, 0], [0, 1, Kv*dt], [0, 0, (1-dt/Tm)]])
        self.B = np.array([[0], [0], [dt/Tm]])
        self.Q = np.array([[0.002, 0, 0], [0, 0.002, 0], [0, 0, 0.002]])
        self.H = np.array([[1, 0, 0], [0, 1, 0]])
        self.R = np.array([[0.12, 0], [0, 0.12]])
        self.x = np.array([[0], [0], [0]])
        self.P = np.eye(3)
        self.I = np.eye(3)

    def predict(self, u):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(self.I - np.dot(K, self.H), self.P)

class MODEL:
    def __init__(self):
        self.mAc = [0.0]
        self.mVi = [0.0]
        self.mFi = [0.0]
        self.mTm = 0.1
        self.mTv = 4
        self.mTf = 0.2
        self.Tmi = 0.01
        self.Tiv = 0.1

        #Parameters for the Kalman filter
        self.kf = KalmanFilter(dt, 1/self.mTf, 1/self.mTv, self.mTm)
        self.last_u = 0.0

    def model(self, dt, u):

        mTms = 0.0
        mTms += dt

        # Noise:
        A = 0.2*np.sin(470*mTms + 60) + 0.1*np.sin(125*mTms)
        noise = random.uniform(-10, 10)

        # Plant dynamics with noise
        intert(dt, u, 1, self.mTm, self.mAc)
        integr(dt, self.mAc[0] + A, self.mTv, self.mVi)
        integr(dt, self.mVi[0], self.mTf, self.mFi)

        # True values with noise
        sensor_Fi = self.mFi[0] + noise + A
        sensor_Vi = self.mVi[0] + noise + A
        sensor_Ac = self.mAc[0] + noise

        # Kalman filter update
        z = np.array([[sensor_Fi], [sensor_Vi]]) 
        self.kf.predict(np.array([[u]]))
        self.kf.update(z)
        filtered_Fi = self.kf.x[0, 0]
        filtered_Vi = self.kf.x[1, 0]
        return sensor_Fi, sensor_Vi, filtered_Fi, filtered_Vi

    def simulate(self, step, dt, F_set, pid_F, pid_V):
        F_true = np.zeros(step)
        V_true = np.zeros(step)
        F_filtered = np.zeros(step)
        V_filtered = np.zeros(step)
        V_setpoints = np.zeros(step)
        time_arr = np.zeros(step)
        u_arr = np.zeros(step)
        
        u_M = [0.0]  
        u_CH = [0.0] 

        for i in range(step - 1):
            # Outer loop: Force PID (using filtered values)
            V_setpoint = pid_F.compute(F_set, F_filtered[i], dt)
            V_setpoints[i] = V_setpoint

            # Inner loop: Velocity PID (using filtered values)
            u = pid_V.compute(V_setpoint, V_filtered[i], dt)
            u = np.clip(u, -1000, 1000)

            # Calculate u_M using intert
            intert(dt, u, 1, self.mTm, u_M)
            u_M[0] = np.clip(u_M[0], -1000, 1000) 

            # Calculate u_CH using integr
            integr(dt, u_M[0], self.mTv, u_CH) 
            u_CH[0] = np.clip(u_CH[0], -1000, 1000) 

            # Calculate final control signal
            u_PID = u_CH[0] + u_M[0]  

            u_arr[i] = u_PID

            # Get plant outputs
            F_true[i+1], V_true[i+1], F_filtered[i+1], V_filtered[i+1] = self.model(dt, u_PID)
            time_arr[i+1] = time_arr[i] + dt

        return F_true, V_true, F_filtered, V_filtered, V_setpoints, time_arr, u_arr


class PIDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.last_error = 0
        self.derivative = [0.0]  # Initialize derivative as a list for rldiff
        self.Tdm = 0.1  # Time constant for derivative calculation

    def compute(self, F, F_set, dt):
        error = F - F_set
        rldiff(dt, error - self.last_error, 1, self.Tdm, self.derivative)
        
        self.last_error = error
        return self.Kp * error + self.Kd * self.derivative[0]

class RealTimePlotter:
    def __init__(self):
        plt.ion()
        self.fig, self.axes = plt.subplots(6, 1, figsize=(12, 18))

        # Initialize plots
        self.cost_line, = self.axes[0].plot([], [], 'b-')

        # PID Parameters
        self.f_lines = [
            self.axes[1].plot([], [], 'r-', label='Kp_F')[0],
            self.axes[1].plot([], [], 'b-', label='Kd_F')[0]
        ]
        self.f_text = self.axes[1].text(0.02, 0.85, '', transform=self.axes[1].transAxes,
                                      bbox=dict(facecolor='white', alpha=0.8), fontsize=9)

        self.v_lines = [
            self.axes[2].plot([], [], 'r-', label='Kp_V')[0],
            self.axes[2].plot([], [], 'b-', label='Kd_V')[0]
        ]
        self.v_text = self.axes[2].text(0.02, 0.85, '', transform=self.axes[2].transAxes,
                                      bbox=dict(facecolor='white', alpha=0.8), fontsize=9)

        # Responses
        self.F_true_line, = self.axes[3].plot([], [], 'r-', label='F_n', alpha=0.5)
        self.F_filt_line, = self.axes[3].plot([], [], 'g-', label='F_k')
        self.target_F = self.axes[3].axhline(F_set, color='k', linestyle='--', label='Set')

        self.V_true_line, = self.axes[4].plot([], [], 'r-', label='V_n', alpha=0.5)
        self.V_filt_line, = self.axes[4].plot([], [], 'b-', label='V_k')

        self.U_line, = self.axes[5].plot([], [], 'm-', label='u_PID')

        self._configure_axes()
        self.cost_history = []
        self.param_history_F = []
        self.param_history_V = []

    def _configure_axes(self):
        titles = ['Cost Function', 'PID Angle', 'PID Velocity',
                 'F Response', 'V Response', 'Control Signal']
        ylabels = ['Cost', 'Parameter Value', 'Parameter Value', 
                  'F (угол)', 'Velocity (m/s)', 'Control (u)']

        for ax, title, ylabel in zip(self.axes, titles, ylabels):
            ax.set_title(title)
            ax.set_xlabel('Iteration' if 'Parameters' in title else 'Time (s)')
            ax.set_ylabel(ylabel)
            ax.grid(True)
            ax.legend()

    def update_plots(self, iteration, cost, params_F, params_V, 
                   F_true, V_true, F_filt, V_filt, V_setpoints, time_array, u_arr):
        self.cost_history.append(cost)
        self.param_history_F.append(params_F)
        self.param_history_V.append(params_V)

        # Update parameter displays
        self.f_text.set_text(f'Angle PID:\nKp={params_F[0]:.4f}\nKd={params_F[1]:.4f}')
        self.v_text.set_text(f'Velocity PID:\nKp={params_V[0]:.4f}\nKd={params_V[1]:.4f}')

        # Update cost plot
        self.cost_line.set_data(range(iteration + 1), self.cost_history)
        self.axes[0].relim()
        self.axes[0].autoscale_view()

        # Update parameter plots
        for i, line in enumerate(self.f_lines):
            line.set_data(range(iteration + 1), np.array(self.param_history_F)[:, i])
        for i, line in enumerate(self.v_lines):
            line.set_data(range(iteration + 1), np.array(self.param_history_V)[:, i])

        self.axes[1].relim()
        self.axes[1].autoscale_view()
        self.axes[2].relim()
        self.axes[2].autoscale_view()

        # Update response plots
        self.F_true_line.set_data(time_array, F_true)
        self.F_filt_line.set_data(time_array, F_filt)
        self.V_true_line.set_data(time_array, V_true)
        self.V_filt_line.set_data(time_array, V_filt)
        self.U_line.set_data(time_array[:-1], u_arr[:-1])

        for ax in self.axes[3:]:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(0.01)

def plant_cost_function(a, return_response=False):
    pid_F = PIDController(a[0], a[1])
    pid_V = PIDController(a[2], a[3])

    F_true, V_true, F_filt, V_filt, V_setpoints, time_arr, u_arr = MODEL().simulate(step, dt, F_set, pid_F, pid_V)

    # Cost based on filtered values
    cost = np.trapezoid((F_set - F_filt)**2, time_arr)/tf

    if return_response:
        return cost, (F_true, V_true, F_filt, V_filt, V_setpoints), time_arr, (None, None), (np.array(a[:2]), np.array(a[2:]), u_arr)
    return cost

class GradientDescent:
    def __init__(self, a, learning_rate, cost_function, a_min=None, a_max=None, plotter=None):
        self.a = np.array(a)
        self.lr = learning_rate
        self.cost_func = cost_function
        self.a_min = np.array(a_min) if a_min else np.full_like(a, -np.inf)
        self.a_max = np.array(a_max) if a_max else np.full_like(a, np.inf)
        self.G = np.zeros_like(a)
        self.cost_history = []
        self.plotter = plotter
        self.iteration = 0

    def grad(self, a):
        h = 1e-2
        cost_at_a = self.cost_func(a)
        return np.array([(self.cost_func(a + h * np.eye(len(a))[i]) - self.cost_func(a - h * np.eye(len(a))[i])) / (2 * h) for i in range(len(a))])


    def execute_adagrad(self, iterations):
        for _ in range(iterations):
            cost, responses, time_array, _, params = self.cost_func(self.a, True)
            self.cost_history.append(cost)

            u_arr = None
            if len(params) == 3:
                params_F, params_V, u_arr = params
            else:
                params_F, params_V = params
                u_arr = np.zeros_like(time_array)

            if self.plotter:
                self.plotter.update_plots(self.iteration, cost, params_F, params_V, 
                                        *responses[:5], time_array, u_arr)

            grad = self.grad(self.a)
            self.G += grad**2
            self.a -= self.lr/(np.sqrt(self.G) + 1e-8) * grad
            self.a = np.clip(self.a, self.a_min, self.a_max)
            self.iteration += 1

def main():
    # Initial PID parameters [Kp_F, Ki_F, Kd_F, Kp_V, Ki_V, Kd_V]
    initial_params = [1.95, 0.2, 1.5, 0.5]

    plotter = RealTimePlotter()
    gd = GradientDescent(initial_params, 0.1, plant_cost_function,
                        [0]*4, [10]*4, plotter)
    gd.execute_adagrad(300)

    plt.ioff()
    plt.show()
    print(f"\nOptimized Parameters:")
    print(f"Angle PID: Kp={gd.a[0]:.4f}, Kd={gd.a[1]:.4f}")
    print(f"Velocity PID: Kp={gd.a[2]:.4f}, Kd={gd.a[3]:.4f}")

if __name__ == "__main__":
    main()
