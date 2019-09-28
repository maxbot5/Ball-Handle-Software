import numpy as np
from numpy import genfromtxt

class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x) #das gemessne x aus der Matrix
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

def example():
	dt = 1.0/50.0
	#F = np.array([[1, dt, 0], [0, 1, dt],[0, 0, 1]]) #state transition model, A
	F = np.array([[1, dt, 0], [0, 1, dt],[0, 0, 1]]) 
	H = np.array([1, 0, 0]).reshape(1, 3) #transponieren #observation model C
	"""
	das ist meine C matrix für den Ausgang, also müsste das mittlere die geschwindigkeit sein
	"""
	q=0.05
	#Q = np.array([[0, 0, 0], [0, 0, 0], [0, 0, q]])
	Q = np.array([[q, q, 0], [q, q, 0], [0, 0, 0]])
	#Q = np.array([[0.05, 0.05, 0.05], [0.05, 0.05, 0.05], [0.05, 0.05, 0.05]])
	#Q = np.array([[0.10, 0.10, 0.0], [0.10, 0.10, 0.0], [0.0, 0.0, 0.0]]) #process noise
	R = np.array([0.4]).reshape(1, 1) #observation noise
	#x = np.linspace(-10, 10, 100)
	#measurements = - (x**2 + 2*x - 2)  + np.random.normal(0, 2, 100)
	myData = genfromtxt('pixy_190602_084414.csv', delimiter=',')
	measurements = myData[150:1000,1:5] # read x value from file
#	measurements = measurements /1.63840 #mm/s^2 
	kf = KalmanFilter(F = F, H = H, Q = Q, R = R)
	predictions = []
	velocity = []
	position = []
	acceleration = []
	for z in measurements:
		predictions.append(np.dot(H,  kf.predict())[0])
		kf.update(z)
		acceleration.append(kf.x[2])
		velocity.append(kf.x[1])
		position.append(kf.x[0])
		print(kf.x[0],kf.x[1], kf.x[2])

	print(kf.x)
	import matplotlib.pyplot as plt
	#plt.plot(range(len(measurements)), measurements, label = 'Measurements')
	#plt.plot(range(len(predictions)), np.array(predictions), label = 'Kalman Filter Prediction')
#	plt.plot(range(len(acceleration)), np.array(acceleration), label = 'Acceleration')
	#plt.plot(range(len(velocity)), np.array(velocity), label = 'Velocity')
	plt.plot(range(len(position)), np.array(position), label = 'Position')
	plt.legend()
	plt.show()

if __name__ == '__main__':
    example()
