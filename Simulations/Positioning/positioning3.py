import numpy as np
import quaternion

def time_update(q, P, y_w, sigma_w):
	q_prime = q * quaternion.from_rotation_vector((dt/2) * y_w)
	G = dt*quaternion.as_rotation_matrix(q_prime)
	Q = sigma_w
	P_prime = P + G@Q@G.T

	return q_prime, P_prime

def measurement_update(q, p_prime, y, sigma_a):
	y_hat = -quaternion.as_rotation_matrix(q)@g

	epsilon  = y - y_hat

	H = -quaternion.as_rotation_matrix(q)@cross_matrix(g)

	R = sigma_a

	S = H@P@H.T + R

	K = P@H.T@np.linalg.inv(S)

	eta = K@epsilon
	q = quaternion.from_rotation_vector(eta/2)* q
	p = p_prime - K@S@K.T

	return q, p

def cross_matrix(a):
	return np.array([[0, -a[2], a[1]],[a[2], 0, -a[0]],[-a[1], a[0], 0]])

dt = 0.01
g = np.array([0, 0, -9.80665])

q = np.quaternion(1, 0, 0, 0)
q_ref = q
P = np.array([[0,0,0],[0,0,0],[0,0,0]])
sigma_w = np.array([[5E-4,0,0],[0,5E-4,0],[0,0,5E-4]])
y = np.array([0,0,0])
sigma_a = np.array([[5E-4,0,0],[0,5E-4,0],[0,0,5E-4]])

for i in range(0, 1000):

	y_w = np.array([2*np.pi,0,0]) + np.random.multivariate_normal([0,0,0], sigma_w)

	q_ref = q_ref * quaternion.from_rotation_vector((dt/2) * np.array([2*np.pi,0,0]))
	q_prime, p_prime = time_update(q, P, y_w, sigma_w)
	q, P = measurement_update(q_prime, p_prime, y, sigma_a)

print(q)
print(q_ref)
