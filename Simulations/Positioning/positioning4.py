from pykalman import KalmanFilter
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import quaternion

def time_update_q(q, P, y_w, sigma_w):
  q_prime = q * quaternion.from_rotation_vector((dt/2) * y_w)
  G = dt*quaternion.as_rotation_matrix(q_prime)
  Q = sigma_w
  P_prime = P + G@Q@G.T

  return q_prime, P_prime

def measurement_update_q(q, p_prime, y, sigma_a):
  y_hat = -quaternion.as_rotation_matrix(q)@g

  epsilon  = y - y_hat

  H = -quaternion.as_rotation_matrix(q)@cross_matrix(g)

  R = sigma_a

  S = H@P_q@H.T + R

  K = P_q@H.T@np.linalg.inv(S)

  eta = K@epsilon
  q = quaternion.from_rotation_vector(eta/2)* q
  p = p_prime - K@S@K.T

  return q, p

def cross_matrix(a):
  return np.array([[0, -a[2], a[1]],[a[2], 0, -a[0]],[-a[1], a[0], 0]])

def time_update(x, P, F, Q):
  x_prime = F@x
  P_prime = F@P@F.T + Q

  return x_prime, P_prime

def measurement_update(x_prime, P_prime, z, H, R):
  y_prime = z-H@x_prime
  S = R+H@P_prime@H.T
  print(S)
  K = P_prime@H.T@np.linalg.inv(S)
  x = x_prime+K@y_prime
  P = (np.eye(9)-K@H)@P_prime@(np.eye(9)-K@H).T + K@R@K.T

  return x, P


accX_data = genfromtxt('accX_data.csv', delimiter=',')
accY_data = genfromtxt('accY_data.csv', delimiter=',')
accZ_data = genfromtxt('accZ_data.csv', delimiter=',')

AccX_Variance = 0.0007
AccY_Variance = 0.0007
AccZ_Variance = 0.0007

# time step
dt = 0.01
g = np.array([0, 0, -9.80665])

# transition_matrix  
F = np.array([[1, dt, 0.5*dt**2, 0,  0,         0, 0,  0,         0], 
     [0,  1,        dt, 0,  0,         0, 0,  0,         0],
     [0,  0,         1, 0,  0,         0, 0,  0,         0],
     [0,  0,         0, 1, dt, 0.5*dt**2, 0,  0,         0],
     [0,  0,         0, 0,  1,        dt, 0,  0,         0],
     [0,  0,         0, 0,  0,         1, 0,  0,         0],
     [0,  0,         0, 0,  0,         0, 1, dt, 0.5*dt**2],
     [0,  0,         0, 0,  0,         0, 0,  1,        dt],
     [0,  0,         0, 0,  0,         0, 0,  0,         1]])

# observation_matrix   
H = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 1]])

G = np.array([[0.5*(dt**2), 0, 0],
              [dt, 0, 0],
              [0, 0.5*(dt**2), 0],
              [0, dt, 0],
              [0, 0, 0.5*(dt**2)],
              [0,0, dt]])

# transition_covariance 
Q = np.array([[0.2,   0,     0,    0,    0,      0,    0,    0,      0],
              [  0, 0.1,     0,    0,    0,      0,    0,    0,      0],
              [  0,   0, 10e-4,    0,    0,      0,    0,    0,      0],
              [  0,   0,     0,  0.2,    0,      0,    0,    0,      0],
              [  0,   0,     0,    0,  0.1,      0,    0,    0,      0],
              [  0,   0,     0,    0,    0,  10e-4,    0,    0,      0],
              [  0,   0,     0,    0,    0,      0,  0.2,    0,      0],
              [  0,   0,     0,    0,    0,      0,    0,  0.1,      0],
              [  0,   0,     0,    0,    0,      0,    0,    0,  10e-4]])

# observation_covariance 
R = np.array([[0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0, AccX_Variance, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0, AccY_Variance, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0,             0],
              [0, 0,             0, 0,             0, 0, 0, 0, AccZ_Variance]])

# initial_state_mean
X0 = np.array([0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0])

# initial_state_covariance
P0 = np.array([[  0,    0,               0, 0, 0,             0, 0, 0,             0], 
      [  0,    0,               0, 0, 0,             0, 0, 0,             0],
      [  0,    0,   AccX_Variance, 0, 0,             0, 0, 0,             0],
      [  0,    0,               0, 0, 0,             0, 0, 0,             0],
      [  0,    0,               0, 0, 0,             0, 0, 0,             0], 
      [  0,    0,               0, 0, 0, AccY_Variance, 0, 0,             0],
      [  0,    0,               0, 0, 0,             0, 0, 0,             0],
      [  0,    0,               0, 0, 0,             0, 0, 0,             0],
      [  0,    0,               0, 0, 0,             0, 0, 0, AccZ_Variance]])

n_timesteps = 30001
n_dim_state = 9
filtered_state_means = np.zeros((n_timesteps, n_dim_state))
filtered_state_covariances = np.zeros((n_timesteps, n_dim_state, n_dim_state))

q = np.quaternion(1, 0, 0, 0)
q_ref = q
P_q = np.array([[0,0,0],[0,0,0],[0,0,0]])
sigma_w_q = np.array([[5E-4,0,0],[0,5E-4,0],[0,0,5E-4]])
y_q = np.array([0,0,0])
sigma_a_q = np.array([[5E-4,0,0],[0,5E-4,0],[0,0,5E-4]])

# iterative estimation for each new measurement
for t in range(n_timesteps):

    y_w_q = np.array([2*np.pi,0,0]) + np.random.multivariate_normal([0,0,0], sigma_w_q)

    q_ref = q_ref * quaternion.from_rotation_vector((dt/2) * np.array([2*np.pi,0,0]))
    q_prime, p_prime_q = time_update_q(q, P_q, y_w_q, sigma_w_q)
    q, P_q = measurement_update_q(q_prime, p_prime_q, y_q, sigma_a_q)

    print(q)
    print(q_ref)

    if t == 0:
        filtered_state_means[t] = X0
        filtered_state_covariances[t] = P0
    else:
        x_prime, P_prime = time_update(filtered_state_means[t-1],
            filtered_state_covariances[t-1], F, Q)

        filtered_state_means[t], filtered_state_covariances[t] = measurement_update(x_prime, P_prime, 
            np.array([0, 0, accX_data[t][4] , 0, 0,  accY_data[t][4], 0, 0, accZ_data[t][4]]), H, R)

f, axarr = plt.subplots(3, 3, sharex=True)

Time = accX_data[:, 0]

axarr[0][0].plot(Time, accX_data[:, 1], label="Input AccX")
axarr[0][0].plot(Time, filtered_state_means[:, 2], "r-", label="Estimated AccX")
axarr[0][0].set_title('Acceleration X')
axarr[0][0].grid()
axarr[0][0].legend()
axarr[0][0].set_ylim([-4, 4])

axarr[1][0].plot(Time, accX_data[:, 2], label="Reference VelX")
axarr[1][0].plot(Time, filtered_state_means[:, 1], "r-", label="Estimated VelX")
axarr[1][0].set_title('Velocity X')
axarr[1][0].grid()
axarr[1][0].legend()
axarr[1][0].set_ylim([-1, 20])

axarr[2][0].plot(Time, accX_data[:, 3], label="Reference PosX")
axarr[2][0].plot(Time, filtered_state_means[:, 0], "r-", label="Estimated PosX")
axarr[2][0].set_title('Position X')
axarr[2][0].grid()
axarr[2][0].legend()
axarr[2][0].set_ylim([-10, 5000])

axarr[0][1].plot(Time, accY_data[:, 1], label="Input AccY")
axarr[0][1].plot(Time, filtered_state_means[:, 5], "r-", label="Estimated AccY")
axarr[0][1].set_title('Acceleration Y')
axarr[0][1].grid()
axarr[0][1].legend()
axarr[0][1].set_ylim([-1, 1])

axarr[1][1].plot(Time, accY_data[:, 2], label="Reference VelY")
axarr[1][1].plot(Time, filtered_state_means[:, 4], "r-", label="Estimated VelY")
axarr[1][1].set_title('Velocity Y')
axarr[1][1].grid()
axarr[1][1].legend()
axarr[1][1].set_ylim([0, 2])

axarr[2][1].plot(Time, accY_data[:, 3], label="Reference PosY")
axarr[2][1].plot(Time, filtered_state_means[:, 3], "r-", label="Estimated PosY")
axarr[2][1].set_title('Position Y')
axarr[2][1].grid()
axarr[2][1].legend()
axarr[2][1].set_ylim([0, 250])

axarr[0][2].plot(Time, accZ_data[:, 1], label="Input AccZ")
axarr[0][2].plot(Time, filtered_state_means[:, 8], "r-", label="Estimated AccZ")
axarr[0][2].set_title('Acceleration Z')
axarr[0][2].grid()
axarr[0][2].legend()
axarr[0][2].set_ylim([-1, 1])

axarr[1][2].plot(Time, accZ_data[:, 2], label="Reference VelZ")
axarr[1][2].plot(Time, filtered_state_means[:, 7], "r-", label="Estimated VelZ")
axarr[1][2].set_title('Velocity Z')
axarr[1][2].grid()
axarr[1][2].legend()
axarr[1][2].set_ylim([-1, 0])

axarr[2][2].plot(Time, accZ_data[:, 3], label="Reference PosZ")
axarr[2][2].plot(Time, filtered_state_means[:, 6], "r-", label="Estimated PosZ")
axarr[2][2].set_title('Position Z')
axarr[2][2].grid()
axarr[2][2].legend()
axarr[2][2].set_ylim([-200, 0])

mse_distX = (np.square(filtered_state_means[:, 0] - accX_data[:, 3])).mean()
mse_distY = (np.square(filtered_state_means[:, 3] - accY_data[:, 3])).mean()
mse_distZ = (np.square(filtered_state_means[:, 6] - accZ_data[:, 3])).mean()

print(f"MSE X-Posititon: No Filter = 4.6617, Filter = {mse_distX}")
print(f"MSE Y-Posititon: No Filter = 3.2600, Filter = {mse_distY}")
print(f"MSE Z-Posititon: No Filter = 1.3355, Filter = {mse_distZ}")
plt.show()