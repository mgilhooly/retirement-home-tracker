import numpy as np
import quaternion
import Accelerometer
import Gyroscope

def time_update(q, P, y_w, Q):
	q_prime = q * quaternion.from_euler_angles((dt/2) * y_w)

	G = dt*quaternion.as_rotation_matrix(q)
	P_prime = P + G@Q@G.T

	return q_prime, P_prime

def measurement_update(q_prime, P_prime, y, sigma_a):
	y_hat = -quaternion.as_rotation_matrix(q_prime)@g

	epsilon  = y - y_hat
	H = -quaternion.as_rotation_matrix(q_prime)@cross_matrix(g)
	R = sigma_a

	S = H@P_prime@H.T + R

	K = P_prime@H.T@np.linalg.inv(S)

	eta = K@epsilon

	q = quaternion.from_euler_angles(eta/2) * q_prime
	P = P_prime - K@S@K.T
	return q, P

def cross_matrix(a):
	return np.array([[0, -a[2], a[1]],
					[a[2], 0, -a[0]],
					[-a[1], a[0], 0]])


gyro = Gyroscope.Gyroscope(5e-4,5e-4,5e-4,0,0,0)
acc = Accelerometer.Accelerometer(5e-4,5e-4,5e-4,0,0,0)
dt = 0.01
g = np.array([0, 0, -9.80665])

q = np.quaternion(1, 0, 0, 0)

P = np.array(np.zeros((3,3)))

y_w_ref = np.array([0,0,0])
sigma_w = np.array([[5E-4,0,0],
					[0,5E-4,0],
					[0,0,5E-4]])

y_a_ref = np.array([0.1,0,0])
sigma_a = np.array([[5E-4,0,0],
					[0,5E-4,0],
					[0,0,5E-4]])

p_simple = np.array([0,0,0])
v_simple = np.array([0,0,0])
p_ref = np.array([0,0,0])
v_ref = np.array([0,0,0])
q_ref = np.quaternion(1, 0, 0, 0)

for i in range(0, 1000):
	p_ref = p_ref + dt*v_ref + 0.5*(dt**2)*y_a_ref
	v_ref = v_ref + dt*y_a_ref
	q_ref = q_ref * quaternion.from_euler_angles(dt*y_w_ref)

	y_a = acc.get_acceleration(y_a_ref)
	y_w = gyro.get_rotation(y_w_ref)

	q_prime, P_prime = time_update(q, P, y_w, sigma_w)
	q, P = measurement_update(q_prime, P_prime, y_a, sigma_a)

	p_simple = p_simple + dt*v_simple + 0.5*(dt**2)*(quaternion.as_rotation_matrix(q)@y_a)
	v_simple = v_simple + dt*(quaternion.as_rotation_matrix(q)@y_a)

print(p_simple)
print(v_simple)
print(quaternion.as_euler_angles(q))

print(p_ref)
print(v_ref)
print(quaternion.as_euler_angles(q_ref))
