#!/usr/bin/env python3
"""
person_simulator.py

Simple ROS2 node to simulate a walking person for the follow_user package.

Features:
- Publishes a PointStamped on /clicked_point at a fixed rate (default 10 Hz)
- Moves along a configurable path (list of (x,y) points). Path may be loaded from
  a CSV file if "path_file" parameter is set, otherwise a small default path is used.
- Adds a smooth random forward offset (1-3 m by default) so the clicked point is
  slightly ahead on the path, simulating a walking person that the AMR should follow.
- Subscribes to /follow_user/target_velocity, /follow_user/current_velocity,
  /follow_user/speed_scale and /cmd_vel and logs their values together with
  the published clicked_point to a CSV file for offline analysis.

Usage:
  python3 person_simulator.py
  ros2 run follow_user person_simulator.py

Parameters (ROS2 params supported by node):
  publish_rate: Hz (default 10.0)
  offset_min: meters (default 1.0)
  offset_max: meters (default 3.0)
  offset_smooth_alpha: smoothing alpha for offset low-pass (default 0.1)
  log_file: CSV file path (default ./person_sim_log.csv)
  frame_id: header frame for PointStamped (default: map)
  path_file: optional path CSV file (x,y per line) to load points from

This node is intentionally simple and robust. It will keep publishing clicked
points even if other nodes/topics are not present yet.
"""

from __future__ import annotations

import csv
import json
import math
import random
import time
from typing import List, Tuple
import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PointStamped, Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros
from tf2_ros import TransformException


def load_path_from_json(path_file: str) -> List[Tuple[float, float]]:
	pts: List[Tuple[float, float]] = []
	try:
		with open(path_file, 'r') as f:
			data = json.load(f)
			# accept either {'path_points': [...]} or raw list
			if isinstance(data, dict) and 'path_points' in data:
				arr = data['path_points']
			else:
				arr = data

			for item in arr:
				if isinstance(item, (list, tuple)) and len(item) >= 2:
					pts.append((float(item[0]), float(item[1])))
	except Exception:
		pts = []
	return pts


class PersonSimulator(Node):
	def __init__(self):
		super().__init__('person_simulator')

		# parameters
		self.declare_parameter('publish_rate', 10.0)
		self.declare_parameter('offset_min', 1.0)
		self.declare_parameter('offset_max', 3.0)
		self.declare_parameter('offset_smooth_alpha', 0.1)
		
		script_dir = os.path.dirname(os.path.realpath(__file__))
		default_log_file = os.path.join(script_dir, 'person_sim_log.csv')
		self.declare_parameter('log_file', default_log_file)
		self.declare_parameter('frame_id', 'slamware_map')
		
		default_path_file = os.path.join(script_dir, 'path.json')
		self.declare_parameter('path_file', default_path_file)
		self.declare_parameter('odom_topic', '/slamware_ros_sdk_server_node/odom')

		self.publish_rate = float(self.get_parameter('publish_rate').value)
		self.offset_min = float(self.get_parameter('offset_min').value)
		self.offset_max = float(self.get_parameter('offset_max').value)
		self.offset_alpha = float(self.get_parameter('offset_smooth_alpha').value)
		self.log_file = str(self.get_parameter('log_file').value)
		self.frame_id = str(self.get_parameter('frame_id').value)
		self.path_file = str(self.get_parameter('path_file').value)
		self.odom_topic = str(self.get_parameter('odom_topic').value)

		# Load or use default path
		path: List[Tuple[float, float]] = []
		if self.path_file:
			path = load_path_from_json(self.path_file)

			if not path:
				self.get_logger().warning(f'Failed to load path from {self.path_file}, using default path')

				if not path:
					# default path (short) if nothing provided
					path = [
						(-0.07, 0.68), (-0.12, 0.68), (-0.17, 0.68), (-0.22, 0.69), (-0.27, 0.69),
						(-0.32, 0.69), (-0.37, 0.70), (-0.42, 0.70), (-0.47, 0.70), (-0.52, 0.71),
						(-0.57, 0.71), (-0.62, 0.71), (-0.67, 0.71), (-0.72, 0.72), (-0.77, 0.72),
						(-0.82, 0.72), (-0.87, 0.73), (-0.92, 0.73), (-0.97, 0.73), (-1.02, 0.74),
						(-1.07, 0.74), (-1.12, 0.74), (-1.17, 0.75), (-1.22, 0.75), (-1.27, 0.75),
						(-1.32, 0.75), (-1.37, 0.76), (-1.42, 0.76), (-1.47, 0.76), (-1.52, 0.77),
						(-1.57, 0.77), (-1.62, 0.77), (-1.67, 0.78), (-1.72, 0.78), (-1.77, 0.78),
						(-1.82, 0.78), (-1.87, 0.79), (-1.92, 0.79), (-1.97, 0.79), (-2.02, 0.80),
						(-2.07, 0.80), (-2.12, 0.80), (-2.17, 0.81), (-2.22, 0.81), (-2.27, 0.81),
					]

		self.path = path

		# simulation state: progress along the path [t between 0..len(path)-1]
		self.segment_idx = 0
		self.segment_t = 0.0

		# base walking speed along path (m/s) - parameterizable later if needed
		self.walk_speed = 1.0

		# smoothed random forward offset (meters)
		self.offset_smoothed = (self.offset_min + self.offset_max) / 2.0

		qos = QoSProfile(depth=10)
		self.clicked_pub = self.create_publisher(PointStamped, '/clicked_point', qos)
		# subscribe to robot odometry to base simulated person on robot pose
		self.robot_odom = None
		self.create_subscription(Odometry, self.odom_topic, self._cb_odom, qos)

		# subscribers for debug topics we want to log (non-blocking)
		self.target_vel = None
		self.current_vel = None
		self.speed_scale = None
		self.cmd_vel = None

		self.create_subscription(Float64, '/follow_user/target_velocity', self._cb_target_vel, qos)
		self.create_subscription(Float64, '/follow_user/current_velocity', self._cb_current_vel, qos)
		self.create_subscription(Float64, '/follow_user/speed_scale', self._cb_speed_scale, qos)
		self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, qos)

		# prepare CSV logging
		try:
			self.csv_file = open(self.log_file, 'w', newline='')
			self.csv_writer = csv.writer(self.csv_file)
			header = ['time', 'clicked_x', 'clicked_y', 'target_velocity', 'current_velocity', 'speed_scale', 'cmd_vel_lin_x', 'cmd_vel_ang_z', 'pose_x', 'pose_y', 'pose_theta']
			self.csv_writer.writerow(header)
			self.csv_file.flush()
			self.get_logger().info(f'Logging to {self.log_file}')
		except Exception as e:
			self.get_logger().warning(f'Failed to open log file {self.log_file}: {e}')
			self.csv_file = None
			self.csv_writer = None

		# TF buffer and listener
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		# Wait for TF to be ready before starting the simulation
		self.wait_for_tf_timer = self.create_timer(1.0, self.wait_for_tf_callback)

	def wait_for_tf_callback(self):
		try:
			# Check if the transform is available.
			self.tf_buffer.lookup_transform('base_link', 'slamware_map', rclpy.time.Time())
			self.get_logger().info('Transform from "slamware_map" to "base_link" is available. Starting person simulation.')
			
			# If transform is available, cancel this timer and start the main one.
			self.wait_for_tf_timer.cancel()
			period = 1.0 / max(1e-3, float(self.publish_rate))
			self.timer = self.create_timer(period, self.timer_cb)

		except TransformException as ex:
			self.get_logger().warn(f'Could not transform slamware_map to base_link: {ex}. Waiting...')

	# --- subscribers callbacks ---
	def _cb_target_vel(self, msg: Float64) -> None:
		try:
			self.target_vel = float(msg.data)
		except Exception:
			self.target_vel = None

	def _cb_current_vel(self, msg: Float64) -> None:
		try:
			self.current_vel = float(msg.data)
		except Exception:
			self.current_vel = None

	def _cb_speed_scale(self, msg: Float64) -> None:
		try:
			self.speed_scale = float(msg.data)
		except Exception:
			self.speed_scale = None

	def _cb_cmd_vel(self, msg: Twist) -> None:
		self.cmd_vel = msg

	def _cb_odom(self, msg: Odometry) -> None:
		# store latest robot odom (pose in global frame)
		self.robot_odom = msg
		self.robot_pose_x = msg.pose.pose.position.x
		self.robot_pose_y = msg.pose.pose.position.y
		_, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

	def euler_from_quaternion(self, q) -> Tuple[float, float, float]:
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (q.w * q.x + q.y * q.z)
		t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (q.w * q.y - q.z * q.x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)

		t3 = +2.0 * (q.w * q.z + q.x * q.y)
		t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
		yaw_z = math.atan2(t3, t4)

		return roll_x, pitch_y, yaw_z

	# --- helper: compute position along path by advancing segment_t ---
	def advance_along_path(self, dt: float) -> Tuple[float, float]:
		# If odometry is available, use robot base pose as the person's base
		if self.robot_odom is not None:
			p = self.robot_odom.pose.pose.position
			return float(p.x), float(p.y)

		# If at final point, stay there
		if self.segment_idx >= len(self.path) - 1:
			return self.path[-1]

		# Compute current segment start/end
		x0, y0 = self.path[self.segment_idx]
		x1, y1 = self.path[self.segment_idx + 1]
		seg_dx = x1 - x0
		seg_dy = y1 - y0
		seg_len = math.hypot(seg_dx, seg_dy)
		if seg_len <= 1e-6:
			# zero-length segment: advance index
			self.segment_idx += 1
			self.segment_t = 0.0
			return self.advance_along_path(dt)

		# advance by walk_speed * dt along current segment
		advance = self.walk_speed * dt
		remain = (1.0 - self.segment_t) * seg_len
		while advance >= remain:
			# move to the end of this segment and reduce advance
			advance -= remain
			self.segment_idx += 1
			self.segment_t = 0.0
			if self.segment_idx >= len(self.path) - 1:
				return self.path[-1]
			x0, y0 = self.path[self.segment_idx]
			x1, y1 = self.path[self.segment_idx + 1]
			seg_dx = x1 - x0
			seg_dy = y1 - y0
			seg_len = math.hypot(seg_dx, seg_dy)
			remain = seg_len
			if seg_len <= 1e-6:
				continue

		# move within the current segment
		frac = advance / seg_len
		self.segment_t += frac
		sx = x0 + seg_dx * self.segment_t
		sy = y0 + seg_dy * self.segment_t
		return sx, sy

	def compute_forward_point(self, base_x: float, base_y: float) -> Tuple[float, float]:
		# Compute a forward point further along the path by offset distance
		# Find a position offset_distance ahead along the polyline starting at (base_x, base_y)
		offset_distance = random.uniform(self.offset_min, self.offset_max)
		# smooth the offset with a simple low-pass
		self.offset_smoothed = (self.offset_alpha * offset_distance) + (1.0 - self.offset_alpha) * self.offset_smoothed
		remaining = self.offset_smoothed

		# start searching from the closest point along the polyline to (base_x, base_y)
		idx, t = self.find_closest_segment_and_t(base_x, base_y)

		while remaining > 1e-3 and idx < len(self.path) - 1:
			x0, y0 = self.path[idx]
			x1, y1 = self.path[idx + 1]
			seg_dx = x1 - x0
			seg_dy = y1 - y0
			seg_len = math.hypot(seg_dx, seg_dy)
			if seg_len <= 1e-6:
				idx += 1
				t = 0.0
				continue

			# position at t
			pos_x = x0 + seg_dx * t
			pos_y = y0 + seg_dy * t
			dist_to_end = seg_len * (1.0 - t)
			if remaining <= dist_to_end:
				# point lies within this segment
				frac = remaining / seg_len
				fx = pos_x + seg_dx * frac
				fy = pos_y + seg_dy * frac
				return fx, fy
			else:
				# consume the rest of this segment and move to next
				remaining -= dist_to_end
				idx += 1
				t = 0.0

		# if we run out of path, return final point
		return self.path[-1]

	def find_closest_segment_and_t(self, x: float, y: float) -> Tuple[int, float]:
		# returns (segment_index, t) where t is fraction [0..1] along segment
		best_idx = 0
		best_t = 0.0
		best_dist = float('inf')
		for i in range(0, len(self.path) - 1):
			x0, y0 = self.path[i]
			x1, y1 = self.path[i + 1]
			dx = x1 - x0
			dy = y1 - y0
			seg_len2 = dx*dx + dy*dy
			if seg_len2 == 0.0:
				continue
			# projection t
			t = ((x - x0)*dx + (y - y0)*dy) / seg_len2
			t_clamped = max(0.0, min(1.0, t))
			px = x0 + dx * t_clamped
			py = y0 + dy * t_clamped
			d2 = (px - x)**2 + (py - y)**2
			if d2 < best_dist:
				best_dist = d2
				best_idx = i
				best_t = t_clamped
		return best_idx, best_t

	def timer_cb(self) -> None:
		now = self.get_clock().now().to_msg()
		dt = 1.0 / max(1e-3, float(self.publish_rate))

		# advance simulated person
		px, py = self.advance_along_path(dt)

		# compute clicked (forward) point
		fx, fy = self.compute_forward_point(px, py)

		# publish
		ps = PointStamped()
		ps.header.stamp = now
		ps.header.frame_id = self.frame_id
		ps.point = Point(x=float(fx), y=float(fy), z=0.0)
		self.clicked_pub.publish(ps)

		# log latest values to CSV if open
		if self.csv_writer is not None:
			lin_x = self.cmd_vel.linear.x if (self.cmd_vel is not None) else ''
			ang_z = self.cmd_vel.angular.z if (self.cmd_vel is not None) else ''
			row = [time.time(), fx, fy, self.target_vel if self.target_vel is not None else '', self.current_vel if self.current_vel is not None else '', self.speed_scale if self.speed_scale is not None else '', lin_x, ang_z,
				self.robot_pose_x if self.robot_pose_x is not None else '',
				self.robot_pose_y if self.robot_pose_y is not None else '',
				self.robot_pose_theta if self.robot_pose_theta is not None else '']
			try:
				self.csv_writer.writerow(row)
				self.csv_file.flush()
			except Exception:
				pass

	def destroy_node(self):
		# close CSV file if open
		try:
			if getattr(self, 'csv_file', None):
				self.csv_file.close()
		except Exception:
			pass
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = PersonSimulator()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

