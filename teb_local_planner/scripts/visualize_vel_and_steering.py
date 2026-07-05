#!/usr/bin/env python3
"""Live plot of v, omega and the implied steering angle from the TEB feedback topic.

rclpy port of visualize_velocity_profile.py with an added steering subplot:
phi = atan(wheelbase * omega / v) (+-pi/2 for pure rotation), the quantity the
teb_flickering_protection node reacts to. Guide lines mark its 1.2 rad trigger
threshold. Requires `publish_feedback: True` on the TEB profile.

Usage:
  ros2 run teb_local_planner visualize_vel_and_steering.py \
      [--feedback-topic /teb_feedback] [--wheelbase 1.2526]
"""

import argparse
import math
import threading

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from teb_msgs.msg import FeedbackMsg


class FeedbackPlotter(Node):
    def __init__(self, topic: str, wheelbase: float):
        super().__init__("visualize_vel_and_steering")
        self.wheelbase = wheelbase
        self.lock = threading.Lock()
        self.data = None
        self.create_subscription(FeedbackMsg, topic, self.callback, 5)

    def callback(self, msg: FeedbackMsg):
        if not msg.trajectories:
            return
        trajectory = msg.trajectories[msg.selected_trajectory_idx].trajectory
        t, v, omega, phi = [], [], [], []
        prev_phi = 0.0
        for point in trajectory:
            t.append(point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)
            vx = point.velocity.linear.x
            wz = point.velocity.angular.z
            v.append(vx)
            omega.append(wz)
            if vx == 0.0 and wz != 0.0:
                prev_phi = math.copysign(math.pi / 2.0, wz)
            elif vx != 0.0:
                prev_phi = math.atan(self.wheelbase * wz / vx)
            phi.append(prev_phi)
        with self.lock:
            self.data = (t, v, omega, phi)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feedback-topic", default="/teb_feedback")
    parser.add_argument("--wheelbase", type=float, default=1.2526)
    args = parser.parse_args()

    rclpy.init()
    node = FeedbackPlotter(args.feedback_topic, args.wheelbase)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.ion()
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle("TEB planned velocity + implied steering profile")

    while rclpy.ok():
        with node.lock:
            data = node.data
        if data is not None:
            t, v, omega, phi = data
            for ax in axes:
                ax.clear()
            axes[0].step(t, v, where="post")
            axes[0].set_ylabel("v [m/s]")
            axes[1].step(t, omega, where="post")
            axes[1].set_ylabel("omega [rad/s]")
            axes[2].step(t, phi, where="post")
            axes[2].set_ylabel("implied phi [rad]")
            axes[2].set_xlabel("time from start [s]")
            for guide in (1.2, -1.2):
                axes[2].axhline(guide, color="gray", lw=0.5, ls="--")
            axes[2].set_ylim(-1.8, 1.8)
        plt.pause(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
