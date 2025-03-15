import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    waypoints = np.loadtxt("angles.txt", delimiter=",") 
    waypoints_len = waypoints.shape[0]
    num_angles = waypoints.shape[1]

    time_arr = np.arange(0, waypoints_len/2, 0.5)

    figure = plt.figure
    for i in range(num_angles):
        plt.plot(time_arr, waypoints[:,i], label="Joint %i" %(i+1))
    
    plt.title("Time vs Joint Angles")
    plt.xlabel("Time (sec)")
    plt.ylabel("Angle (radians)")
    plt.legend(loc='upper right')
    plt.show()