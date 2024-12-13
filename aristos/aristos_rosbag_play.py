#!/usr/bin/env python
import os

home = os.getenv('HOME')
bag = '/rosbag_tests_domokos/teleoperation_ground_truth_2024-11-26-15-28-47.bag'
path = home+bag
topics = [
    '/aristos/imu/data',
    '/aristos/filter/positionlla/gnss',
    '/livox/imu',
    '/livox/lidar',
]

topics_str = ' '.join(topics)
cmd = "rosbag play "+path+" --topics " + topics_str

os.system(cmd)