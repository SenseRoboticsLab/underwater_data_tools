import rosbag
import cv2
import numpy as np
import glob
import os
import csv
import argparse
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from waterlinked_a50_ros_driver.msg import DVL

output_dir = ''

def save_compressed_image_to_png(msg, name, count):
    # Convert the compressed image to PNG format
    np_arr = np.frombuffer(msg.data, np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    # Create a filename
    image_filename = f"{output_dir}/{name}/{timestamp}.png"
    if not os.path.exists(f"{output_dir}/{name}"):
        os.makedirs(f"{output_dir}/{name}")
    # Save the image
    cv2.imwrite(image_filename, img_np)
    # print(f"Saved image: {image_filename}")


def save_gt_to_csv(msg, csv_writer):
    # Extract relevant data from Odometry message
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    # Write the position and orientation data to CSV
    csv_writer.writerow([timestamp, position.x, position.y, position.z,
                         orientation.x, orientation.y, orientation.z, orientation.w])


def save_depth_to_csv(msg, csv_writer):
    # Extract relevant data from Odometry message
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    position = msg.pose.pose.position
    # Write the position and orientation data to CSV
    csv_writer.writerow([timestamp, position.z])


def save_imu_to_csv(msg, csv_writer):
    # Extract relevant data from IMU message
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    orientation = msg.orientation
    angular_velocity = msg.angular_velocity
    linear_acceleration = msg.linear_acceleration
    # Write IMU data to CSV
    csv_writer.writerow([timestamp, angular_velocity.x, angular_velocity.y, angular_velocity.z,
                         linear_acceleration.x, linear_acceleration.y, linear_acceleration.z])


def save_dvl_to_csv(msg, csv_writer):
    # Extract DVL data
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    velocity = msg.velocity
    beams = msg.beams

    csv_writer.writerow([timestamp, velocity.x, velocity.y, velocity.z, beams[0].velocity, beams[1].velocity, beams[2].velocity, beams[3].velocity])


def convert_rosbag_to_png_and_csv(input_bag):
    with rosbag.Bag(input_bag, 'r') as bag:
        # Prepare CSV files
        gt_csv_file = open(f"{output_dir}/gt.csv", mode='w', newline='')
        gt_full_csv_file = open(f"{output_dir}/gt_full.csv", mode='w', newline='')
        aqua_slam_csv_file = open(f"{output_dir}/aqua_slam_pose.csv", mode='w', newline='')
        depth_csv_file = open(f"{output_dir}/depth.csv", mode='w', newline='')
        imu_csv_file = open(f"{output_dir}/imu.csv", mode='w', newline='')
        dvl_csv_file = open(f"{output_dir}/dvl.csv", mode='w', newline='')

        # CSV writers
        gt_csv_writer = csv.writer(gt_csv_file)
        gt_full_csv_writer = csv.writer(gt_full_csv_file)
        aqua_slam_csv_writer = csv.writer(aqua_slam_csv_file)
        depth_csv_writer = csv.writer(depth_csv_file)
        imu_csv_writer = csv.writer(imu_csv_file)
        dvl_csv_writer = csv.writer(dvl_csv_file)

        # Write CSV headers
        gt_csv_writer.writerow(['timestamp[ns]', 'pos_x[m]', 'pos_y[m]', 'pos_z[m]', 'orient_qx', 'orient_qy', 'orient_qz', 'orient_qw'])
        gt_full_csv_writer.writerow(['timestamp[ns]', 'pos_x[m]', 'pos_y[m]', 'pos_z[m]', 'orient_qx', 'orient_qy', 'orient_qz', 'orient_qw'])
        aqua_slam_csv_writer.writerow(['timestamp[ns]', 'pos_x[m]', 'pos_y[m]', 'pos_z[m]', 'orient_qx', 'orient_qy', 'orient_qz', 'orient_qw'])
        depth_csv_writer.writerow(['timestamp[ns]', 'pos_z[m]'])
        imu_csv_writer.writerow(['timestamp[ns]', 'ang_vel_x[rad/s]', 'ang_vel_y[rad/s]', 'ang_vel_z[rad/s]',
                                 'lin_acc_x[m/s^2]', 'lin_acc_y[m/s^2]', 'lin_acc_z[m/s^2]'])
        dvl_csv_writer.writerow(['timestamp[ns]', 'vel_x[m/s]', 'vel_y[m/s]', 'vel_z[m/s]', 'beam1_vel[m/s]', 'beam2_vel[m/s]', 'beam3_vel[m/s]', 'beam4_vel[m/s]'])

        image_count_left = 0
        image_count_right = 0

        for topic, msg, t in bag.read_messages():
            if topic == '/camera/left/image_dehazed/compressed':
                image_count_left += 1
                save_compressed_image_to_png(msg, 'IMG_L', image_count_left)
            elif topic == '/camera/right/image_dehazed/compressed':
                image_count_right += 1
                save_compressed_image_to_png(msg, 'IMG_R', image_count_right)
            elif topic == '/apriltag_slam/GT':
                save_gt_to_csv(msg, gt_csv_writer)
            elif topic == '/apriltag_slam/GT_full':
                save_gt_to_csv(msg, gt_full_csv_writer)
            elif topic == '/aqua_slam/pose':
                save_gt_to_csv(msg, aqua_slam_csv_writer)
            elif topic == '/depth/data':
                save_depth_to_csv(msg, depth_csv_writer)
            elif topic == '/imu/data':
                save_imu_to_csv(msg, imu_csv_writer)
            elif topic == '/dvl/data':
                save_dvl_to_csv(msg, dvl_csv_writer)

        # Close CSV files
        gt_csv_file.close()
        gt_full_csv_file.close()
        aqua_slam_csv_file.close()
        depth_csv_file.close()
        imu_csv_file.close()
        dvl_csv_file.close()

        print("Conversion to PNG and CSV completed.")


if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Convert ROS bag files to PNG images and CSV files.')
    parser.add_argument('bag_dir', type=str, help='Directory containing .bag files to process')
    args = parser.parse_args()

    bag_dir = args.bag_dir
    bag_files = glob.glob(os.path.join(bag_dir, '*.bag'))
    print(f"found {len(bag_files)} bag files, processing...")

    for input_bag in bag_files:
        directory = os.path.dirname(input_bag)
        filename = os.path.splitext(os.path.basename(input_bag))[0]
        output_dir = os.path.join(directory, filename)

        # Create directories if they don't exist
        os.makedirs(output_dir, exist_ok=True)

        convert_rosbag_to_png_and_csv(input_bag)