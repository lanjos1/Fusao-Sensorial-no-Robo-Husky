#!/usr/bin/env python
import csv
import math
import os

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationMetrics(object):
    def __init__(self):
        self.run_name = rospy.get_param("~run_name", "odom")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/catkin_ws/src/localizacao_husky/results"))
        self.filtered_topic = rospy.get_param("~filtered_topic", "/odometry/filtered")
        self.gt_topic = rospy.get_param("~gt_topic", "/gt/odom")
        self.max_pair_dt = rospy.Duration(rospy.get_param("~max_pair_dt", 0.1))
        self.gt_offset_x = rospy.get_param("~gt_offset_x", 0.0)
        self.gt_offset_y = rospy.get_param("~gt_offset_y", 0.0)
        
        self.latest_gt = None
        self.rows = []
        self.sum_sq_pos = 0.0
        self.sum_sq_yaw = 0.0
        self.count = 0
        self.final_position_error = 0.0
        self.final_yaw_error = 0.0

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        rospy.Subscriber(self.gt_topic, Odometry, self.gt_cb, queue_size=50)
        rospy.Subscriber(self.filtered_topic, Odometry, self.filtered_cb, queue_size=50)
        rospy.on_shutdown(self.write_results)

    def gt_cb(self, msg):
        self.latest_gt = msg

    def filtered_cb(self, msg):
        if self.latest_gt is None:
            return
        
        dt = abs(msg.header.stamp - self.latest_gt.header.stamp)
        if msg.header.stamp != rospy.Time() and self.latest_gt.header.stamp != rospy.Time() and dt > self.max_pair_dt:
            return

        fx = msg.pose.pose.position.x
        fy = msg.pose.pose.position.y
        gx = self.latest_gt.pose.pose.position.x + self.gt_offset_x
        gy = self.latest_gt.pose.pose.position.y + self.gt_offset_y

        pos_error = math.hypot(fx - gx, fy - gy)
        yaw_error = wrap_angle(yaw_from_quaternion(msg.pose.pose.orientation) -
                               yaw_from_quaternion(self.latest_gt.pose.pose.orientation))

        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        
        self.rows.append([self.run_name, stamp, fx, fy, gx, gy, pos_error, yaw_error])
        
        self.sum_sq_pos += pos_error ** 2
        self.sum_sq_yaw += yaw_error ** 2
        self.count += 1
        self.final_position_error = pos_error
        self.final_yaw_error = yaw_error

    def write_results(self):
        if not self.rows:
            rospy.logwarn("Sem amostras de metricas coletadas para o run '%s'", self.run_name)
            return

        csv_path = os.path.join(self.output_dir, "todos_os_modos_metrics.csv")
        plot_path = os.path.join(self.output_dir, "%s_Trajetoria_e_erro.png" % self.run_name)

        file_exists = os.path.isfile(csv_path)

        with open(csv_path, "a") as csv_file:
            writer = csv.writer(csv_file)
            if not file_exists:
                writer.writerow(["mode", "time", "filtered_x", "filtered_y", "gt_x", "gt_y", "position_error", "yaw_error_rad"])
            writer.writerows(self.rows)

        self.write_plot(plot_path)
        rospy.loginfo("Metrics for mode '%s' appended to %s", self.run_name, csv_path)

    def write_plot(self, plot_path):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            rospy.logwarn("matplotlib nao esta disponivel; Pulando PNG")
            return

        t0 = self.rows[0][1] 
        times = [row[1] - t0 for row in self.rows]
        errors = [row[6] for row in self.rows]
        yaw_errors = [row[7] for row in self.rows]  
        fx = [row[2] for row in self.rows]
        fy = [row[3] for row in self.rows]
        gx = [row[4] for row in self.rows]
        gy = [row[5] for row in self.rows]

     
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        
        axes[0].plot(gx, gy, label="gt", color="red", linestyle="--")
        axes[0].plot(fx, fy, label="filtered_%s" % self.run_name)
        axes[0].set_title("Trajetoria: Modo %s" % self.run_name)
        axes[0].set_xlabel("x [metros]")
        axes[0].set_ylabel("y [metros]")
        axes[0].axis("equal")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(times, errors, color="tab:blue")
        axes[1].set_title("Erro de posicao: Modo %s" % self.run_name)
        axes[1].set_xlabel("tempo [segundos]")
        axes[1].set_ylabel("erro [metros]")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(times, yaw_errors, color="tab:orange")
        axes[2].set_title("Erro de yaw (orientacao): Modo %s" % self.run_name)
        axes[2].set_xlabel("tempo [segundos]")
        axes[2].set_ylabel("erro [radianos]")
        axes[2].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=140)
        plt.close(fig)


if __name__ == "__main__":
    rospy.init_node("localization_metrics")
    LocalizationMetrics()
    rospy.spin()