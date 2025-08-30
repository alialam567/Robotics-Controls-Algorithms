import numpy as np
import math
import matplotlib.pyplot as plt

# Feedback control, lets try coding feedforward control as well and then the feedforward with feedback adjustment like ECE486
# Going to program first the case where I dont care about my final pose angle and then where I do care about my final pose angle
class PID():
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.cum_angle_error = 0
        self.prev_angle_error = 0

    def adjust(self, current_pose, goal_pose):
        dist_error = np.sqrt((current_pose.x - goal_pose.x)**2 + (current_pose.y - goal_pose.y)**2)
        desired_angle = np.arctan2((goal_pose.y - current_pose.y),(goal_pose.x - current_pose.x))
        angle_error = desired_angle - current_pose.theta
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi #wrap angle between -pi to pi /(-180 to 180)deg
        self.cum_angle_error += angle_error * dt
        d_error = (angle_error - self.prev_angle_error)/dt
        w = (self.kp * angle_error) + (self.ki * self.cum_angle_error) + (self.kd * d_error)
        self.prev_angle_error = angle_error
        return w
        

class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def positional_error(self, other):
        dx = other.x - self.x
        dy = other.y - self.y
        error_vector = (dx, dy)
        error_magnitude = math.hypot(dx, dy)
        return error_vector, error_magnitude



# Simulation stuff
dt = 0.1
T = 300
steps = int(T/dt)

#Robot state variables
x, y, theta = 0.0, 0.0, 0.0

#Controls
v = 0.2  # m/s
#w = 0.2  # rad/s

#Robot path
xs, ys = [], []
current_pose = Pose(0.0, 0.0, 0.0)
goal_pose = Pose(-5.0, 5.0, 0.0)


#Controller
pid = PID(kp=0.7, kd=0.05, ki=0.01)

for step in range(steps):
    # Adjust step
    w = pid.adjust(current_pose, goal_pose)
    # Kinematics
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt

    current_pose.x = x
    current_pose.y = y
    current_pose.theta = theta

    xs.append(x)
    ys.append(y)

    err_vec, err_mag = current_pose.positional_error(goal_pose)

    if (err_mag < 0.1):
        break 


# Plotting
plt.figure(figsize=(6,6))
plt.plot(xs, ys, label="Trajectory")
plt.quiver(xs[-1], ys[-1], np.cos(theta), np.sin(theta), scale=6, color="r")
plt.quiver(goal_pose.x, goal_pose.y, np.cos(goal_pose.theta), np.sin(goal_pose.theta), scale=6, color="g")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.legend()
plt.show()