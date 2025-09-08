#!/usr/bin/env python3
"""
plot_log.py

Simple utility to plot person simulator logs saved by person_simulator.py
Saves PNGs into the same folder as the CSV.

Usage:
  python3 plot_log.py src/follow_user/test/person_sim_log.csv

Dependencies: matplotlib, numpy
"""
from __future__ import annotations

import os
import sys
import csv
import math
from typing import List

import matplotlib.pyplot as plt
import numpy as np


def read_log(csv_path: str):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def to_float(x):
    try:
        return float(x)
    except Exception:
        return math.nan


def plot(csv_path: str):
    rows = read_log(csv_path)
    if not rows:
        print('No rows in CSV')
        return

    t = [to_float(r.get('time')) for r in rows]
    clicked_x = [to_float(r.get('clicked_x')) for r in rows]
    clicked_y = [to_float(r.get('clicked_y')) for r in rows]
    target_v = [to_float(r.get('target_velocity')) for r in rows]
    current_v = [to_float(r.get('current_velocity')) for r in rows]
    speed_scale = [to_float(r.get('speed_scale')) for r in rows]
    cmd_lin = [to_float(r.get('cmd_vel_lin_x')) for r in rows]
    cmd_ang = [to_float(r.get('cmd_vel_ang_z')) for r in rows]

    pose_x = [to_float(r.get('pose_x')) for r in rows]
    pose_y = [to_float(r.get('pose_y')) for r in rows]
    pose_theta = [to_float(r.get('pose_theta')) for r in rows]

    out_dir = os.path.dirname(csv_path) or '.'

    # trajectory
    plt.figure(figsize=(6,6))
    plt.plot(clicked_x, clicked_y, '-o', markersize=3, label='clicked_point')
    plt.plot(pose_x, pose_y, '-o', markersize=3, label='robot_pose')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Clicked point and robot trajectory')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, 'clicked_robot_trajectory.png'))
    plt.close()

    # velocities
    plt.figure()
    plt.plot(t, target_v, label='target_velocity')
    plt.plot(t, current_v, label='current_velocity')
    plt.plot(t, cmd_lin, label='cmd_vel_lin_x')
    plt.xlabel('time (s)')
    plt.ylabel('m/s')
    plt.title('Velocities over time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, 'velocities.png'))
    plt.close()

    # speed scale and angular
    plt.figure()
    plt.plot(t, speed_scale, label='speed_scale')
    plt.plot(t, cmd_ang, label='cmd_vel_ang_z')
    plt.xlabel('time (s)')
    plt.title('Speed scale and angular command')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, 'scale_and_angular.png'))
    plt.close()

    # robot theta
    plt.figure()
    plt.plot(t, pose_theta, label='pose_theta')
    plt.xlabel('time (s)')
    plt.ylabel('rad')
    plt.title('Robot theta over time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, 'pose_theta.png'))
    plt.close()

    print(f'Wrote plots to {out_dir}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: plot_log.py <person_sim_log.csv>')
        sys.exit(1)
    plot(sys.argv[1])
