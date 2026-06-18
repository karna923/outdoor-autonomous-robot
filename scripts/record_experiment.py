import os
import subprocess
import time
import signal
import sys
import argparse

process = None
def main():
  parser = argparse.ArgumentParser(description='takes in experiment arguments')
  parser.add_argument('experiment', choices=['static', 'dynamic', 'mixed'])
  parser.add_argument('--duration', type=int, default=60)
  args = parser.parse_args()
  
  #topic list
  topics = ['/odom', '/odometry/filtered', '/gps', '/imu', '/scan', '/tf', '/tf_static']
  
  timestamp = time.strftime("%Y-%m-%d_%H_%M_%S")
  output_path = f"~/outdoor-autonomous-robot/data/{args.experiment}/{timestamp}"
  output_path = os.path.expanduser(output_path)
  print(f"Recording {args.experiment} experiment to {output_path}")

  
  cmd = ['ros2', 'bag', 'record', '-o', output_path, '--duration', str(args.duration)] + topics
  global process
  signal.signal(signal.SIGINT, signal_handler)
  process = subprocess.Popen(cmd)
  process.wait()


def signal_handler(sig, frame):
  process.terminate()
  sys.exit(0)

if __name__ == '__main__':
    main()
