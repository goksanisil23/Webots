import numpy as np
import matplotlib.pyplot as plt


measurement_file = open('/home/gisil/Work/SLAM/SLAM/analysis/wheel_measurement.txt', 'r') 
ground_truth_file = open('/home/gisil/Work/SLAM/SLAM/analysis/ground_truth.txt', 'r') 

x = []
z = []
yaw = []

x_gt = []
z_gt = []

## parse the log file and populate variables
while True: 
	line_measurement = measurement_file.readline()
	line_gt = ground_truth_file.readline()
	
	decoded_line_measurement = line_measurement.split()
	decoded_line_gt = line_gt.split()

	if(len(decoded_line_measurement) > 1):
		x.append(float(decoded_line_measurement[0]))
		z.append(float(decoded_line_measurement[1]))
		yaw.append(float(decoded_line_measurement[2]))

	if(len(decoded_line_gt) > 1):
		x_gt.append(float(decoded_line_gt[0]))
		z_gt.append(float(decoded_line_gt[1]))

	if not line_measurement:
		break

####### plot the motion

time = [t*0.1 for t in list(range(len(x)))]


plt.plot(x, z, "b", label="trajectory")
plt.plot(x_gt, z_gt, "r", label="GT")
plt.legend()
plt.xlabel("x")
plt.ylabel("z")
plt.grid(True)
plt.show()