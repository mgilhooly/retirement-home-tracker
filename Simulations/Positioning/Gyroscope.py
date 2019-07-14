import numpy as np

class Gyroscope:
	def __init__(self, sigma_x, sigma_y, sigma_z, e_delta_x, e_delta_y, e_delta_z, bits = 16, range = 250):
		self.sigma = np.array([[sigma_x, 0, 0],[0, sigma_y, 0],[0, 0, sigma_z]])
		self.delta = np.array([0,0,0])
		self.e_delta = np.array([[e_delta_x, 0, 0],[0, e_delta_y, 0],[0, 0, e_delta_z]])

	def get_rotation(self, omega):
		e = np.random.multivariate_normal([0,0,0], self.sigma)
		self.delta = self.delta + np.random.multivariate_normal([0,0,0], self.e_delta)
		y = omega + e
		return np.float16(y)

if __name__ == "__main__":
	g = Gyroscope(1,1,1,1,1,1)

	f, (ax1, ax2, ax3) = plt.subplots(3,1)
	for i in range(0, 10):
		omega = g.get_rotation(np.array([0,0,0]))
		print(omega)