import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')
import numpy as np

with open("robot_data.csv") as fil:
    titles = fil.readline()[:-1].split(" ")
    print(titles)

exec("%s = np.loadtxt('robot_data.csv', delimiter=' ', skiprows=1).T"%(",".join(titles)))
target_q = np.vstack([target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5]).T
target_qd = np.vstack([target_qd_0, target_qd_1, target_qd_2, target_qd_3, target_qd_4, target_qd_5]).T
target_qdd = np.vstack([target_qdd_0, target_qdd_1, target_qdd_2, target_qdd_3, target_qdd_4, target_qdd_5]).T
actual_q = np.vstack([actual_q_0, actual_q_1, actual_q_2, actual_q_3, actual_q_4, actual_q_5]).T
actual_qd = np.vstack([actual_qd_0, actual_qd_1, actual_qd_2, actual_qd_3, actual_qd_4, actual_qd_5]).T
actual_current = np.vstack([actual_current_0, actual_current_1, actual_current_2, actual_current_3, actual_current_4, actual_current_5]).T
joint_control_output = np.vstack([joint_control_output_0, joint_control_output_1, joint_control_output_2, joint_control_output_3, joint_control_output_4, joint_control_output_5]).T
actual_TCP_pose = np.vstack([actual_TCP_pose_0, actual_TCP_pose_1, actual_TCP_pose_2, actual_TCP_pose_3, actual_TCP_pose_4, actual_TCP_pose_5]).T
actual_TCP_speed = np.vstack([actual_TCP_speed_0, actual_TCP_speed_1, actual_TCP_speed_2, actual_TCP_speed_3, actual_TCP_speed_4, actual_TCP_speed_5]).T
actual_TCP_force = np.vstack([actual_TCP_force_0, actual_TCP_force_1, actual_TCP_force_2, actual_TCP_force_3, actual_TCP_force_4, actual_TCP_force_5]).T
target_TCP_pose = np.vstack([target_TCP_pose_0, target_TCP_pose_1, target_TCP_pose_2, target_TCP_pose_3, target_TCP_pose_4, target_TCP_pose_5]).T
target_TCP_speed = np.vstack([target_TCP_speed_0, target_TCP_speed_1, target_TCP_speed_2, target_TCP_speed_3, target_TCP_speed_4, target_TCP_speed_5]).T
joint_temperatures = np.vstack([joint_temperatures_0, joint_temperatures_1, joint_temperatures_2, joint_temperatures_3, joint_temperatures_4, joint_temperatures_5]).T
joint_mode = np.vstack([joint_mode_0, joint_mode_1, joint_mode_2, joint_mode_3, joint_mode_4, joint_mode_5]).T
actual_joint_voltage = np.vstack([actual_joint_voltage_0, actual_joint_voltage_1, actual_joint_voltage_2, actual_joint_voltage_3, actual_joint_voltage_4, actual_joint_voltage_5]).T

print(f"{actual_q[0,:]=}")
print(f"{actual_q[-1,:]=}")

plt.plot(timestamp[:-1]-timestamp[0], timestamp[1:]-timestamp[:-1])

plt.figure()
plt.plot(timestamp-timestamp[0], actual_current[:,0])


plt.figure()

print(timestamp)
plt.plot(timestamp-timestamp[0], target_q[:,0])
plt.plot(timestamp-timestamp[0], actual_q[:,0])
plt.plot(timestamp-timestamp[0], actual_joint_voltage[:,0])
plt.plot(timestamp-timestamp[0], joint_temperatures[:,0])
plt.plot(timestamp-timestamp[0], actual_current[:,0])
plt.show()




# sys.path.append("..")
# import argparse
# import logging
# import signal
# import sys

# import rtde.csv_reader as csv_reader


# class Plotter(object):
#     # load data
#     plot_samples = None
#     plot_data = []
#     number_of_plot_colors = 12
#     color_list = []
#     x = None  # data range

#     def signal_handler(signal, frame):
#         plt.close("all")
#         sys.exit(0)

#     def __init__(self):
#         # parse arguments

#         parser = argparse.ArgumentParser()
#         parser.add_argument("type", help="plot type (x,xd,q,qd,qdd,i,0:5)", nargs="+")
#         parser.add_argument(
#             "--file", default=["robot_data.csv"], help="data file", nargs="+"
#         )
#         parser.add_argument(
#             "--filter",
#             help="exclude data when no program is running",
#             action="store_true",
#         )

#         args = parser.parse_args()

#         logging.basicConfig(level=logging.INFO)

#         plot_types = args.type

#         self.get_plot_data(args)

#         # prepare plots
#         plt.close("all")
#         numberOfPlots = 7
#         background_color = plt.cm.gist_earth(np.random.rand(1))[0]

#         self.x = range(self.plot_samples)

#         self.color_list = plt.cm.Paired(np.linspace(0, 1, self.number_of_plot_colors))
#         self.plot_all(plot_types, numberOfPlots, background_color)

#         signal.signal(signal.SIGINT, self.signal_handler)

#     def get_plot_color(self, style, cnt):
#         if cnt < 0:
#             cnt = 0
#         if cnt >= self.number_of_plot_colors:
#             cnt = self.number_of_plot_colors - 1
#         if "r" in style:
#             return self.color_list[cnt * 2]
#         if "b" in style:
#             return self.color_list[cnt * 2 + 1]
#         return self.color_list[self.number_of_plot_colors - 1 - cnt]

#     def makesubplot_withdata(self, subplot, y, name, style, y_range=6, color=None):
#         if color is None:
#             (axis,) = subplot.plot(
#                 self.x[0 : self.plot_samples], y[0 : self.plot_samples], style
#             )
#         else:
#             (axis,) = subplot.plot(
#                 self.x[0 : self.plot_samples],
#                 y[0 : self.plot_samples],
#                 style,
#                 color=color,
#             )
#         axis.set_label(name)
#         subplot.set_ylim([-y_range, y_range])

#     def makesubplot(self, subplot, name, style, y_range=6):
#         plot_name = name
#         cnt = 0
#         for p in self.plot_data:
#             print(p.__dict__.keys())
#             y = p.__dict__[name]
#             if len(self.plot_data) > 1:
#                 plot_name = name + " " + plt.get_name()
#             plot_color = self.get_plot_color(style, cnt)
#             self.makesubplot_withdata(subplot, y, plot_name, style, y_range, plot_color)
#             cnt = cnt + 1

#     def addYtext(self, subplots, textArray):
#         for pl in range(len(subplots)):
#             subplots[pl].set_ylabel(textArray[pl])
#         return subplots

#     def plot_all(self, plot_types, numberOfPlots, background_color):
#         for plot_type in plot_types:
#             f, subplots = plt.subplots(numberOfPlots, num="Plotting %s"%plot_type, sharex=True, sharey=False)
#             # tmp_window_title = f.canvas.get_window_title()
#             f.set_facecolor(background_color)

#             if plot_type == "q":
#                 f.suptitle("Q", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": Q")
#                 naming = [
#                     "base",
#                     "shoulder",
#                     "elbow",
#                     "wrist 1",
#                     "wrist 2",
#                     "wrist 3",
#                     "state",
#                 ]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_" + plot_type + "_" + str(i)
#                     self.makesubplot(subplots[i], name, "rx-")
#                     name = "actual_" + plot_type + "_" + str(i)
#                     self.makesubplot(subplots[i], name, "b+-")

#             elif plot_type == "i":
#                 f.suptitle("I", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": I")
#                 naming = [
#                     "base",
#                     "shoulder",
#                     "elbow",
#                     "wrist 1",
#                     "wrist 2",
#                     "wrist 3",
#                     "state",
#                 ]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_current_" + str(i)
#                     target_current = self.plot_data[0].__dict__[name]
#                     self.makesubplot(subplots[i], name, "rx-")
#                     name = "actual_current_" + str(i)
#                     self.makesubplot(subplots[i], name, "b+-")
#                     name = "actual_current_window_" + str(i)
#                     current_window = self.plot_data[0].__dict__[name]
#                     self.makesubplot_withdata(
#                         subplots[i],
#                         target_current + current_window,
#                         "current max",
#                         "--",
#                     )
#                     self.makesubplot_withdata(
#                         subplots[i],
#                         target_current - current_window,
#                         "current min",
#                         "--",
#                     )

#             elif plot_type == "qd":
#                 f.suptitle("QD", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": QD")
#                 naming = [
#                     "base",
#                     "shoulder",
#                     "elbow",
#                     "wrist 1",
#                     "wrist 2",
#                     "wrist 3",
#                     "state",
#                 ]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_" + plot_type + "_" + str(i)
#                     self.makesubplot(subplots[i], name, "rx-")
#                     name = "actual_" + plot_type + "_" + str(i)
#                     self.makesubplot(subplots[i], name, "b+-")

#             elif plot_type == "qdd":
#                 f.suptitle("QDD", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": QDD")
#                 naming = [
#                     "base",
#                     "shoulder",
#                     "elbow",
#                     "wrist 1",
#                     "wrist 2",
#                     "wrist 3",
#                     "state",
#                 ]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_qdd_" + str(i)
#                     self.makesubplot(subplots[i], name, "rx-", 40)

#             elif plot_type == "x":
#                 f.suptitle("X", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": X")
#                 naming = ["X", "Y", "Z", "XA", "YA", "ZA", "state"]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_TCP_pose_" + str(i)
#                     self.makesubplot(subplots[i], name, "rx-")
#                     name = "actual_TCP_pose_" + str(i)
#                     self.makesubplot(subplots[i], name, "b+-")

#             elif plot_type == "xd":
#                 f.suptitle("XD", fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": XD")
#                 naming = ["X", "Y", "Z", "XA", "YA", "ZA", "state"]
#                 self.addYtext(subplots, naming)
#                 for i in range(6):
#                     name = "target_TCP_speed_" + str(i)
#                     self.makesubplot(subplots[i], name, "rx-")
#                     name = "actual_TCP_speed_" + str(i)
#                     self.makesubplot(subplots[i], name, "b+-")

#             elif plot_type.isdigit():
#                 idx = int(plot_type)
#                 if idx < 0 or idx > 6:
#                     raise ValueError("Out of range")
#                 joints = ["base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3"]
#                 f.suptitle("joint: " + joints[idx], fontsize=12)
#                 # f.canvas.set_window_title(tmp_window_title + ": joint " + joints[idx])
#                 naming = [
#                     "q",
#                     "qd",
#                     "qdd",
#                     "current",
#                     "joint mode",
#                     "control output",
#                     "state",
#                 ]
#                 self.addYtext(subplots, naming)
#                 name = "target_q_" + str(idx)
#                 self.makesubplot(subplots[0], name, "rx-")
#                 name = "actual_q_" + str(idx)
#                 self.makesubplot(subplots[0], name, "b+-")
#                 name = "target_qd_" + str(idx)
#                 self.makesubplot(subplots[1], name, "rx-")
#                 name = "actual_qd_" + str(idx)
#                 self.makesubplot(subplots[1], name, "b+-")
#                 name = "target_qdd_" + str(idx)
#                 self.makesubplot(subplots[2], name, "rx-", 40)
#                 name = "target_current_" + str(idx)
#                 target_current = self.plot_data[0].__dict__[name]
#                 self.makesubplot(subplots[3], name, "rx-")
#                 name = "actual_current_" + str(idx)
#                 self.makesubplot(subplots[3], name, "b+-")
#                 name = "actual_current_window_" + str(idx)
#                 current_window = self.plot_data[0].__dict__[name]
#                 self.makesubplot_withdata(
#                     subplots[3], target_current + current_window, "current max", "--"
#                 )
#                 self.makesubplot_withdata(
#                     subplots[3], target_current - current_window, "current min", "--"
#                 )
#                 name = "joint_mode_" + str(idx)
#                 self.makesubplot(subplots[4], name, "b+-")
#                 name = "joint_control_output_" + str(idx)
#                 self.makesubplot(subplots[5], name, "b+-")

#             else:
#                 raise ValueError("Unrecognized plot type: " + plot_type)

#             self.makesubplot(subplots[6], "robot_mode", "rx-", 10)
#             self.makesubplot(subplots[6], "safety_mode", "bx-", 10)

#             for i in range(numberOfPlots):
#                 legend = subplots[i].legend(
#                     loc="upper right", shadow=True, fontsize="x-small"
#                 )

#         plt.show()

#     def fill_plot_data(self, data, plot_samples, plot_data):
#         if plot_samples is None or data.get_samples() < plot_samples:
#             plot_samples = data.get_samples()
#         plot_data.append(data)
#         return (plot_samples, plot_data)

#     def get_plot_data(self, args):

#         # np.loadtxt(args.file[0]):


#         for file in args.file:
#             with open(file) as csvfile:
#                 data = csv_reader.CSVReader(csvfile, filter_running_program=args.filter)
#                 self.plot_samples, self.plot_data = self.fill_plot_data(
#                     data, self.plot_samples, self.plot_data
#                 )
#                 print(self.plot_samples)
#                 print(self.plot_data)


# if __name__ == "__main__":
#     Plotter()
