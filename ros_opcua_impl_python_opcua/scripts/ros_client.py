#!/usr/bin/env python
# from http import client
from lib2to3.pytree import Node
import sys
import time

#import rosgraph
#import rosnode


import rospy
#from opcua import Server, ua
from opcua import Client


class SubHandler(object):
    """
    """
    def datachange_notification(self, node, val, data):
        print("Python: New data change event", node, val)
        print("getting parent")
        node.call_method()
        # parent = node.get_parent()
        # print(parent)
        print("parent printed")


class ROSClient:
    def __init__(self, addr):
        self.server_addr = addr
        self.client = Client(addr)  

    def connect_to_server(self):
        try:
            self.client.connect()
        except:
            print("errorrrrr")
        print("Client is connected to server!!!!!")

    def subscribe(self):
        node_str = 'ns=2;s=/teststation/controller/activate_cutter/command'
        node = self.client.get_node(node_str)
        handler = SubHandler()
        self.sub = self.client.create_subscription(1, handler)
        self.sub.subscribe_data_change(node)
        print("subscribed")


# def main(args):
#     rospy.init_node("ros_opcua_client_node")

#     print("clientttttttttttttt haha")

#     client = Client("opc.tcp://127.0.0.1:21554/")
#     # rospy.sleep(2)
#     try:
#         client.connect()
#     except:
#         pass

#     print("connected!")

#     root = client.get_root_node()


# if __name__ == "__main__":
#     main(sys.argv)
