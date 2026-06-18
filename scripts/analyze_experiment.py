import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def main():
  parser = argparse.ArgumentParser(description='analyzes data')
  parser.add_argument('experiment', choices=['static', 'dynamic', 'mixed'])
  parser.add_argument('bag_path')
  args = parser.parse_args()
  
  reader = rosbag2_py.SequentialReader()
  storage_options = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id='sqlite3')
  converter_options = rosbag2_py.ConverterOptions('', '')
  reader.open(storage_options, converter_options)
  
  odom_data = {'x': [], 'y': [], 't': []}
  gps_data = {'x': [], 'y': [], 't': []}
  ekf_data = {'x': [], 'y': [], 't': []}

  while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/odom':
      msg = deserialize_message(data, Odometry)
      odom_data['x'].append(msg.pose.pose.position.x)
      odom_data['y'].append(msg.pose.pose.position.y)
      odom_data['t'].append(timestamp)
    elif topic == '/odometry/gps': 
      msg = deserialize_message(data, Odometry)
      gps_data['x'].append(msg.pose.pose.position.x)
      gps_data['y'].append(msg.pose.pose.position.y)
      gps_data['t'].append(timestamp)
    elif topic == '/odometry/filtered':
      msg = deserialize_message(data, Odometry)
      ekf_data['x'].append(msg.pose.pose.position.x)
      ekf_data['y'].append(msg.pose.pose.position.y)
      ekf_data['t'].append(timestamp)
  
  odom_data['x'] = np.array(odom_data['x'])
  odom_data['y'] = np.array(odom_data['y'])
  odom_data['t'] = np.array(odom_data['t'])
  
  gps_data['x'] = np.array(gps_data['x'])
  gps_data['y'] = np.array(gps_data['y'])
  gps_data['t'] = np.array(gps_data['t'])
  
  ekf_data['x'] = np.array(ekf_data['x'])
  ekf_data['y'] = np.array(ekf_data['y'])
  ekf_data['t'] = np.array(ekf_data['t'])
  ekf_data['t'] = ekf_data['t'] - ekf_data['t'][0]
  
  odom_x_interp = np.interp(ekf_data['t'], odom_data['t'], odom_data['x'])
  odom_y_interp = np.interp(ekf_data['t'], odom_data['t'], odom_data['y'])
  
  gps_x_interp = np.interp(ekf_data['t'], gps_data['t'], gps_data['x'])
  gps_y_interp = np.interp(ekf_data['t'], gps_data['t'], gps_data['y'])

  odom_ekf_div = np.hypot(ekf_data['x'] - odom_x_interp, ekf_data['y'] - odom_y_interp)
  gps_ekf_div = np.hypot(ekf_data['x'] - gps_x_interp, ekf_data['y'] - gps_y_interp)
  
  plt.plot(ekf_data['t'], odom_ekf_div, label='odom vs EKF')
  plt.plot(ekf_data['t'], gps_ekf_div, label='GPS vs EKF')
  
  plt.xlabel('Time (s)')
  plt.ylabel('Divergence (m)')
  plt.title(f'{args.experiment} — EKF divergence')
  plt.show()
  
if __name__ == '__main__':
  main()