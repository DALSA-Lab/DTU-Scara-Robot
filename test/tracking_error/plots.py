import matplotlib.pyplot as plt
import pandas as pd


# Load data
# data_CM = pd.read_csv('test/tracking_error/record_0p15707_0p157_j1_vel_p0p5_i0p0_d0p0_kff1p0_CM.csv')
data_pid = pd.read_csv('test/tracking_error/record_0p314_0p157_j1_vel_p0p5_i0p0_d0p0_kff1p0.csv')
data_ff = pd.read_csv('test/tracking_error/record_0p314_0p157_j1_vel_p0p0_i0p0_d0p0_kff1p0.csv')

# print(data_pid.keys())
# Plot
plt.figure(figsize=(6.8, 4.2))
x = [t-data_pid['__time'][0] for t in data_pid['__time']]
# plt.plot(data_CM.iloc[:, 18]) # j1 position
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'],label="q_1")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'],label="q_ref,1")



# plt.xticks(x, data_pid['month'])
# plt.xlabel('Month')
# plt.ylabel('Sales')
plt.legend()

x = [t-data_ff['__time'][0] for t in data_ff['__time']]
plt.figure(figsize=(6.8, 4.2))
plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'],label="q_1")
plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'],label="q_ref,1")

# plt.xticks(x, data_pid['month'])
# plt.xlabel('Month')
# plt.ylabel('Sales')
plt.legend()

x = [t-data_pid['__time'][0] for t in data_pid['__time']]
plt.figure(figsize=(6.8, 4.2))
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/velocities[0]'],label="qd_1")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'],label="qd_ref,1")

# plt.xticks(x, data_pid['month'])
# plt.xlabel('Month')
# plt.ylabel('Sales')
plt.legend()

plt.show()


