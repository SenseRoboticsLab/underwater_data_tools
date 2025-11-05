import glob
import os
import rosbag
import rospy
from nav_msgs.msg import Odometry


def filter_bag(input_bag, output_bag, topics_to_keep):
    """
    Filters the input ROS bag by keeping only the topics in topics_to_keep list.

    :param input_bag: Path to the input bag file.
    :param output_bag: Path to the output bag file.
    :param topics_to_keep: List of topics to retain in the output bag.
    """
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic in topics_to_keep:
                    outbag.write(topic, msg, t)
    print(f"Filtered bag saved as {output_bag}")


def add_depth(input_bag, output_bag):
    depth_topics = '/bluerov2/odometry'
    with rosbag.Bag(output_bag, 'a') as outbag:
        with rosbag.Bag(input_bag, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[depth_topics]):
                if msg._type == 'nav_msgs/Odometry':
                    # Extract the z position from the position field
                    z_position = msg.pose.pose.position.z
                    dep_msg = Odometry()
                    dep_msg.header = msg.header
                    dep_msg.pose.pose.position.z = msg.pose.pose.position.z
                    dep_msg.pose.pose.orientation.w = 1
                    # Set the covariance matrix for the z position (index 14)
                    dep_msg.pose.covariance = [0] * 36  # Initialize a 6x6 matrix with zeros
                    dep_msg.pose.covariance[14] = 0.01  # Set covariance for z position
                    outbag.write('/depth/data', dep_msg, t)


# Example usage
input_dir = '/home/da/extra/data/dataset/bluerov/TRO_dataset/24_10'
output_dir = '/home/da/extra/data/IJRR_dataset'
topics_to_keep = ['/apriltag_slam/GT', '/camera/left/image_dehazed/compressed', '/camera/right/image_dehazed/compressed', '/dvl/data', '/imu/data']


# Search for all .bag files in the directory
bag_files = glob.glob(os.path.join(input_dir, '*.bag'))

# Print and return the full paths
for bag_file in bag_files:
    # print(bag_file)
    print(f"Input: {bag_file}")
    outbag = os.path.join(output_dir,os.path.basename(bag_file))
    filter_bag(bag_file, outbag, topics_to_keep)
    add_depth(bag_file, outbag)
    print(f"Output: {outbag}")


# filter_bag(input_bag, output_bag, topics_to_keep)
# add_depth(input_bag, output_bag)
