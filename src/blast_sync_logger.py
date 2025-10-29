#!/usr/bin/env python

import rospy
from gazebo_blast3d.msg import BlastSync
import csv
import os

csv_path = "/home.md2/sparab2/wind/uncc_wind_control/ros_image/ros_ws/sync_log.csv"

def callback(msg):
    # Write a row to the CSV
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow([
                "event_id", "source", "sim_time", "ros_time", "latency_ms", "vehicle", "standoff_dist", "seq"
            ])
        latency_ms = (msg.ros_time - msg.sim_time) * 1e3
        writer.writerow([
            msg.event_id,
            msg.source,
            msg.sim_time,
            msg.ros_time,
            latency_ms,
            msg.vehicle,
            msg.standoff_dist,
            msg.header.seq
        ])

def main():
    rospy.init_node("blast_sync_logger")
    rospy.Subscriber("/blast_sync_log", BlastSync, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
