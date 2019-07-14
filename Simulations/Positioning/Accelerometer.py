import numpy as np

class Accelerometer:
	g = np.array([0, 0, -9.80665])

	def __init__(self,sigma_x, sigma_y, sigma_z, e_delta_x, e_delta_y, e_delta_z, bits = 16, range = 4):
		self.sigma = np.array([[sigma_x, 0, 0],[0, sigma_y, 0],[0, 0, sigma_z]])
		self.delta = np.array([0,0,0])
		self.e_delta = np.array([[e_delta_x, 0, 0],[0, e_delta_y, 0],[0, 0, e_delta_z]])

	def get_acceleration(self, a):
		e = np.random.multivariate_normal([0,0,0], self.sigma)
		self.delta = self.delta + np.random.multivariate_normal([0,0,0], self.e_delta)
		y = a-self.g + e
		return np.float16(y)

if __name__ == "__main__":
	a = Accelerometer(1,1,1,1,1,1)
	for i in range(0, 10):
		omega = a.get_acceleration(np.array([0,0,0]))

		print(omega)