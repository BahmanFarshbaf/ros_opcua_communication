#!/usr/bin/env python
from multiprocessing.connection import Client
from pydoc import cli
from pydoc_data.topics import topics
import sys
import this
import time
from webbrowser import get

import rosgraph
import rosnode
import rospy
from opcua import Server, ua


import ros_services
import ros_topics


# import ros_client
from opcua import Client

from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs import *

#client = ros_client.ROSClient("opc.tcp://0.0.0.0:21554/")

# Returns the hierachy as one string from the first remaining part on.


def nextname(hierachy, index_of_last_processed):
    try:
        output = ""
        counter = index_of_last_processed + 1
        while counter < len(hierachy):
            output += hierachy[counter]
            counter += 1
        return output
    except Exception as e:
        rospy.logerr("Error encountered ", e)


# def own_rosnode_cleanup():
#     pinged, unpinged = rosnode.rosnode_ping_all()
#     if unpinged:
#         master = rosgraph.Master(rosnode.ID)
#         # noinspection PyTypeChecker
#         rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    def __init__(self):
        self.namespace_ros = rospy.get_param("/rosopcua/namespace")
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        rospy.init_node("rosopcua")
        self.server = Server()
        self.server.set_endpoint("opc.tcp://0.0.0.0:21554/")
        self.server.set_server_name("ROS ua Server")
        self.server.start()

        self.method_map = None

        # setup our own namespaces, this is expected
        uri_topics = "http://ros.org/topics"
        # two different namespaces to make getting the correct node easier for get_node (otherwise had object for service and topic with same name
        #uri_services = "http://ros.org/services"
        uri_actions = "http://ros.org/actions"
        idx_topics = self.server.register_namespace(uri_topics)
        #idx_services = self.server.register_namespace(uri_services)
        idx_actions = self.server.register_namespace(uri_actions)
        # get Objects node, this is where we should put our custom stuff
        objects = self.server.get_objects_node()

        # one object per type we are watching
        topics_object = objects.add_object(idx_topics, "ROS-Topics")
        #services_object = objects.add_object(idx_services, "ROS-Services")
        actions_object = objects.add_object(idx_actions, "ROS_Actions")

        # hh
        self.client = Client("opc.tcp://0.0.0.0:21554/")
        self.client.connect()
        print("client conected")

        # while not rospy.is_shutdown():

        all_topics_lst = self.get_all_topics()

        # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
        # ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, idx_services, services_object)
        self.method_map = ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
                                                                idx_topics, idx_actions, topics_object, actions_object, all_topics_lst)
        # hh

        sub_lst = []
        for key in self.method_map:
            if(key.find("/header") != -1 or key.find("time") != -1):
                del self.method_map[key]
                continue
            leaf_node = self.client.get_node(key)
            sub = self.client.create_subscription(100, self)
            handle = sub.subscribe_data_change(leaf_node)
            sub_lst.append(sub)
            rospy.logdebug(sub_lst)

        print(self.method_map)
        time.sleep(0)  # used to be 60

        # self.server.stop()
        # quit()

    def get_all_topics(self):
        # function to provide topics
        all_ros_topics = []
        all_ros_topics.append(['/teststation/controller/activate_cutter/command', 'std_msgs/Bool'])
        all_ros_topics.append(['/teststation/controller/activate_thread_clamp/command', 'std_msgs/Bool'])
        all_ros_topics.append(['/teststation/controller/open_head/command', 'std_msgs/Bool'])
        all_ros_topics.append(['/teststation/controller/unlock_tool_changer/command', 'std_msgs/Bool'])

        all_ros_topics.append(['/teststation/controller/is_cutter_activated/state', 'std_msgs/ByteMultiArray'])
        #all_ros_topics.append(['/teststation/controller/is_cutter_deactivated/state', 'std_msgs/ByteMultiArray'])
        #all_ros_topics.append(['/teststation/controller/is_head_close/state', 'std_msgs/ByteMultiArray'])
        #all_ros_topics.append(['/teststation/controller/is_head_open/state', 'std_msgs/ByteMultiArray'])
        #all_ros_topics.append(['/teststation/controller/is_needle_cutting/state', 'std_msgs/ByteMultiArray'])
        # all_ros_topics.append(['/teststation/controller/is_needle_top/state', 'std_msgs/ByteMultiArray'])
        # all_ros_topics.append(['/teststation/controller/is_toolchanger_locked/state', 'std_msgs/ByteMultiArray'])
        # all_ros_topics.append(['/teststation/controller/is_toolchanger_unlocked/state', 'std_msgs/ByteMultiArray'])

        return all_ros_topics

    # cb

    def datachange_notification(self, node, val, data):
        nodeid_str = node.nodeid.to_string()
        method_str = self.method_map[nodeid_str]
        method = self.client.get_node(method_str)
        node.call_method(method)

    # def find_service_node_with_same_name(self, name, idx):
    #     rospy.logdebug("Reached ServiceCheck for name " + name)
    #     for service in self.servicesDict:
    #         rospy.logdebug(
    #             "Found name: " + str(self.servicesDict[service].parent.nodeid.Identifier))
    #         if self.servicesDict[service].parent.nodeid.Identifier == name:
    #             rospy.logdebug("Found match for name: " + name)
    #             return self.servicesDict[service].parent
    #     return None

    def find_topics_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached TopicCheck for name " + name)
        for topic in self.topicsDict:
            rospy.logdebug(
                "Found name: " + str(self.topicsDict[topic].parent.nodeid.Identifier))
            if self.topicsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.topicsDict[topic].parent
        return None

    def find_action_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached ActionCheck for name " + name)
        for topic in self.actionsDict:
            rospy.logdebug(
                "Found name: " + str(self.actionsDict[topic].parent.nodeid.Identifier))
            if self.actionsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.actionsDict[topic].parent
        return None


def main(args):
    rosserver = ROSServer()


if __name__ == "__main__":
    main(sys.argv)
