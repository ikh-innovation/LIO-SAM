#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from copy import deepcopy

# Map of message type strings to actual classes
MSG_TYPE_MAP = {
    "std_msgs/Header": Header,
    "sensor_msgs/Imu": Imu,
    "sensor_msgs/NavSatFix": NavSatFix,
    "sensor_msgs/PointCloud2": PointCloud2
}

def relay_callback(msg, args):
    pub, new_frame_id, new_topic_name = args

    # Create a deep copy to avoid modifying the original message
    new_msg = deepcopy(msg)

    # Update the header/frame information
    if hasattr(new_msg, 'header'):
        new_msg.header.frame_id = new_frame_id
        new_msg.header.stamp = rospy.Time.now()

    # Publish the modified message
    pub.publish(new_msg)

def relay_topic(original_topic, new_topic, msg_type, new_frame_id):
    # Create a publisher for the new topic
    pub = rospy.Publisher(new_topic, msg_type, queue_size=10)

    # Create a subscriber for the original topic
    rospy.Subscriber(original_topic, msg_type, relay_callback, (pub, new_frame_id, new_topic))

def load_config(file_path):
    # Load the YAML configuration file
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rospy.init_node('topic_relay_node')

    # Load configuration from a YAML file
    topics_to_relay = rospy.get_param('topics_to_relay')
    # config = load_config(config_file_path)

    # # Set up relays for all specified topics
    # topics_to_relay = config.get('topics_to_relay', [])
    for config in topics_to_relay:
        msg_type_str = config['msg_type']
        msg_type = MSG_TYPE_MAP.get(msg_type_str)

        if not msg_type:
            rospy.logerr("Unsupported message type: {}".format(msg_type_str))
            continue

        relay_topic(
            config['original_topic'],
            config['new_topic'],
            msg_type,
            config['new_frame_id']
        )

    rospy.loginfo("Topic relays are active.")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
