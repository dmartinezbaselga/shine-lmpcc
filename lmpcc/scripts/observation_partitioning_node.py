#!/usr/bin/env python3

# This node is intended to read observations of the robot, outputting a partition from which the data should come

import math
import numpy as np
from collections import deque
import rospy, rospkg

# Necessary for Python3 relative imports (as far as I know)
import os, sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(script_dir))

from geometry_msgs.msg import Point
from derived_object_msgs.msg import ObjectArray
from lmpcc_msgs.msg import observation_partitioning
import pickle
import logging
from numpy import random
from scripts.partition import Partitioner, Observables, ObservablestoJson
from time import time
import warnings

global classfier
global object_past_motion
object_past_motion = dict()


def NormalizeData(data, index):
    min_data = [-9.937835693359375, -7.478607177734375, -3.141588501317783, -1.470312476158142, -1.470312476158142,
                -1.0622188704354423, -0.3726562431880406, -0.31404594012669157, 0.0014749999999999997,
                -0.4344666666666666]
    max_data = [11.746917724609375, 10.0, 3.141592653589793, 1.4699218273162842, 1.470312476158142, 1.08865158898489820,
                2539620442049844, 0.2539620442049844, 1.474, 0.48996666666666666]
    return 2 * (data - min_data[index]) / (max_data[index] - min_data[index]) - 1


def objectCB(msg):
    print('objectCB')
    global classfier
    global object_past_motion
    out_msg = observation_partitioning()
    out_msg.object_ids = []
    out_msg.partitions = []
    history = 7
    out_msg.header = msg.header
    time_to_partition_start = time()
    for object in msg.objects:
        if (object.id not in object_past_motion):
            object_past_motion[object.id] = {"current": True,
                                             "position": deque([], maxlen=history),
                                             "velocity": deque([], maxlen=history),
                                             "obs_vel": deque([], maxlen=history),
                                             "direction": deque([], maxlen=history),
                                             "sidewalk_distance": deque([], maxlen=history),
                                             "crosswalk_distance": deque([], maxlen=history),
                                             "sidewalk_velocity_x": deque([], maxlen=history),
                                             "sidewalk_velocity_y": deque([], maxlen=history)}

        object_past_motion[object.id]["current"] = True
        pos = Point(object.pose.position.x, object.pose.position.y, 0.0)
        object_past_motion[object.id]["position"].append(pos)
        vel = Point(object.twist.linear.x, object.twist.linear.y, 0.0)
        object_past_motion[object.id]["velocity"].append(vel)
        observable = Observables(pos, vel)
        # ii = partitioner.data_to_index(observable, True)
        observable_class = ObservablestoJson()
        direction, sidewalk_distance, crosswalk_distance, _, side, sidewalk_velocity_x, sidewalk_velocity_y, sidewalk_angle = observable_class.all_observables(
            observable)
        observable_velocity = np.sqrt(sidewalk_velocity_x ** 2 + sidewalk_velocity_y ** 2)

        object_past_motion[object.id]["direction"].append(direction)
        object_past_motion[object.id]["sidewalk_distance"].append(sidewalk_distance)
        object_past_motion[object.id]["crosswalk_distance"].append(crosswalk_distance)
        object_past_motion[object.id]["sidewalk_velocity_x"].append(sidewalk_velocity_x)
        object_past_motion[object.id]["sidewalk_velocity_y"].append(sidewalk_velocity_y)
        object_past_motion[object.id]["obs_vel"].append(observable_velocity)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sidewalk_distance = object_past_motion[object.id]["sidewalk_distance"][-1]
            crosswalk_distance = object_past_motion[object.id]["crosswalk_distance"][-1]
            observable_velocity = object_past_motion[object.id]["obs_vel"][-1]
            observable_acceleration = np.mean(np.diff(np.array(object_past_motion[object.id]["obs_vel"])))
            direction = object_past_motion[object.id]["direction"][-1]
            sidewalk_velocity_x = object_past_motion[object.id]["sidewalk_velocity_x"][-1]
            sidewalk_velocity_y = object_past_motion[object.id]["sidewalk_velocity_y"][-1]
            sidewalk_acceleration_x = np.mean(np.diff(object_past_motion[object.id]["sidewalk_velocity_x"]))
            sidewalk_acceleration_y = np.mean(np.diff(object_past_motion[object.id]["sidewalk_velocity_y"]))
            direction_change = np.mean(np.diff(object_past_motion[object.id]["direction"]))
            sidewalk_distace_change = np.mean(np.diff(object_past_motion[object.id]["sidewalk_distance"]))
            features = np.array([sidewalk_distance,
                                 crosswalk_distance,
                                 direction,
                                 sidewalk_velocity_x,
                                 sidewalk_velocity_y,
                                 sidewalk_distace_change,
                                 sidewalk_acceleration_x,
                                 sidewalk_acceleration_y,
                                 observable_velocity,
                                 observable_acceleration
                                 ])

        features = np.reshape(features, (-1, 10))
        weight = 10
        for i in range(features.shape[1]):
            features[:, i] = NormalizeData(features[:, i], i)
            if (i == 2 or i == 4 or i == 1 or i == 8):
                features[:, i] *= 20
            else:
                features[:, i] *= 10

        try:
            ii = classfier.predict(features) # NaN's occur here sometimes when data is not being received
            out_msg.object_ids.append(object.id)
            out_msg.partitions.append(int(ii))
            out_msg.observable_velocity.append(observable_velocity)
            out_msg.angle.append(sidewalk_angle)
        except ValueError:
            pass

        # Enter data into the output msg
        # out_msg.object_ids.append(object.id)
        # out_msg.partitions.append(int(ii))
        # out_msg.observable_velocity.append(observable_velocity)
        # out_msg.angle.append(sidewalk_angle)
        # print('Partitioned {} at {}'.format(out_msg.object_ids[-1], out_msg.partitions[-1]))
        # print(out_msg)
    print('Publish partitions')
    partition_pub.publish(out_msg)
    time_to_partition_end = time() - time_to_partition_start

    print(time_to_partition_end)

    to_rem = []
    for i in object_past_motion:
        if (object_past_motion[i]["current"] == False):
            to_rem.append(i)
        else:
            object_past_motion[i]["current"] = False
    for i in to_rem:
        object_past_motion.pop(i)


if __name__ == '__main__':
    # rospy.loginfo('Pedestrian Module: Initializing')
    # self.walker_count = parameters['spawn_count']
    global classfier
    cwd = os.getcwd()
    classfier = pickle.load(open(rospkg.RosPack().get_path("lmpcc") + "/scripts/" + "model_continous.pkl", "rb"))
    rospy.init_node("observation_partitioning_node", anonymous=True)
    subscribes = rospy.get_param('/subscribe')

    objects_topic = subscribes['obstacles']

    # Initialize the class that partitions the input space
    partitioner = Partitioner()
    # print("HELLLLLLLLLLLLLo")
    # Subscribers
    objects_sub = rospy.Subscriber(objects_topic, ObjectArray, objectCB)
    # print("HELLLLLLLLLLLLLo")
    # Publishers
    partition_pub = rospy.Publisher('/lmpcc/observation_partitions', observation_partitioning, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()
