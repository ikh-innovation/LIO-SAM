#!/usr/bin/env python

import rosbag
from std_msgs.msg import Header
from rospy import Time, get_param
import os
import rospy
import json 
    

def modify_bag(input_bag_path, output_bag_path, topic_frame_id_mapping, topic_pcl_ring_field_mapping, topic_rename_mapping, write_only_specified):
    """
    :param input_bag_path: Path to the input bag file
    :param output_bag_path: Path to the output bag file
    :param topic_frame_id_mapping: Dictionary where keys are topic names and values are new frame_id strings
    :param topic_pcl_ring_field_mapping: Dictionary where keys are topic names and values are new ring field names
    :param topic_prefix: Prefix to add to topic names
    """
    if not os.path.exists(input_bag_path):
        raise FileNotFoundError("Input bag file does not exist: {}".format(input_bag_path))

    with rosbag.Bag(input_bag_path, 'r') as input_bag, rosbag.Bag(output_bag_path, 'w') as output_bag:

        for topic, msg, t in input_bag.read_messages():

            if write_only_specified and topic not in topic_frame_id_mapping and topic not in topic_pcl_ring_field_mapping and topic not in topic_rename_mapping:
                # Skip topics not in the specified mappings if write_only_specified is True
                continue

            # Check if the topic is in the mapping
            if topic in topic_frame_id_mapping:
                # Ensure the message has a header and modify the frame_id
                if hasattr(msg, 'header'): # and isinstance(msg.header, Header):
                    msg.header.frame_id = topic_frame_id_mapping[topic]
                else:
                    print("Warning: Topic {} does not have a header to modify.".format(topic))

            if topic in topic_pcl_ring_field_mapping:
                # Ensure the message has fields and modify the ring field name
                if hasattr(msg, 'fields'):
                    for field in msg.fields:
                        if field.name == "line":
                            field.name = topic_pcl_ring_field_mapping[topic][0]
                    for field in msg.fields:
                        if field.name == "timestamp":
                            field.name = topic_pcl_ring_field_mapping[topic][1]
                else:
                    print("Warning: Topic {} does not have fields to modify.".format(topic))

            # Add the prefix to the topic name
            new_topic = topic
            if topic in topic_rename_mapping:
                new_topic = topic_rename_mapping[topic]

            # Write the message to the output bag
            output_bag.write(new_topic, msg, t)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("modify_bag_node")

    # Retrieve parameters from the parameter server
    input_bag = rospy.get_param("~input_bag")
    output_bag = rospy.get_param("~output_bag")
    topic_frame_id_mapping = json.loads(rospy.get_param("~topic_frame_id_mapping"))
    topic_pcl_ring_field_mapping = json.loads(rospy.get_param("~topic_pcl_ring_field_mapping"))
    topic_rename_mapping = json.loads(rospy.get_param("~topic_rename_mapping"))
    write_only_specified = rospy.get_param("~write_only_specified", False)

    # Run the modification
    modify_bag(input_bag, output_bag, topic_frame_id_mapping, topic_pcl_ring_field_mapping, topic_rename_mapping, write_only_specified)
    # print("Modified bag saved to {}".format(output_bag))
