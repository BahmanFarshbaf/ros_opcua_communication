#!/usr/bin/env python
import sys
import time
import rosgraph
import rosnode
import rospy
from opcua import Server, ua
from opcua.ua import object_ids as o_ids
from datetime import datetime
from threading import Timer

# This function remains unchanged
def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)

# This class remains unchanged
class SubHandler(object):
    def datachange_notification(self, node, val, data):
        print(node.get_display_name().Text, val)

class ROSServer:
    def __init__(self):
        # The line getting the ROS param is now removed.
        rospy.init_node("opcua")
        self.server = Server()
        
        with open('/data/workcell_smp_irb2600/config/irs_opcua_server.txt', 'r') as txt:
            txtfile = txt.read()
            
        self.server.set_endpoint(txtfile)
        self.server.import_xml("/data/workcell_smp_irb2600/config/irs_opcua_nodes_string_ids.xml")
        self.server.start()
        
        # --- CHANGE HERE ---
        # Instead of getting a parameter, we hardcode the exact namespace URI from your XML file.
        namespace_uri = "https://vetrontypical-europe.com/OPCUA/"
        self.idx = self.server.get_namespace_index(namespace_uri)
        
        print("OPC UA Server using namespace index: {}".format(self.idx))

        handler = SubHandler()
        sub = self.server.create_subscription(100, handler)
        
        # --- Browse for all variable nodes ---
        root_node_id = "ns={};s=IRS_MES".format(self.idx)
        root_node = self.server.get_node(root_node_id)
        
        all_variable_nodes = []
        self.browse_and_collect_vars(root_node, all_variable_nodes)
        print("Found {} total variable nodes.".format(len(all_variable_nodes)))

        nodes_to_exclude = [
            "ns={};s=IRS_MES.System.Client.SystemTimeUTC".format(self.idx),
            "ns={};s=IRS_MES.System.Server.SystemTimeUTC".format(self.idx)
        ]

        nodelist = [
            node for node in all_variable_nodes 
            if node.nodeid.to_string() not in nodes_to_exclude
        ]
        
        for var in nodelist:
            var.set_writable()

        print("Subscribing to data changes on {} nodes after exclusions.".format(len(nodelist)))
        sub.subscribe_data_change(nodelist)
        
        server_time_node_id = "ns={};s=IRS_MES.System.Server.SystemTimeUTC".format(self.idx)
        server_time_node = self.server.get_node(server_time_node_id)
        rt = RepeatedTimer(30, timeupdater, server_time_node)

        print("ROS OPCUA Server initialized.")
        while not rospy.is_shutdown():
            rospy.spin()
             
        time.sleep(0)

        self.server.stop()
        rt.stop()
        print("ROS OPCUA Server stopped.")
        quit()

    def browse_and_collect_vars(self, parent_node, collection_list):
        for child_node in parent_node.get_children():
            if child_node.get_node_class() == ua.NodeClass.Variable:
                collection_list.append(child_node)
            self.browse_and_collect_vars(child_node, collection_list)


def timeupdater(node_to_update):
    node_to_update.set_value(ua.Variant(datetime.utcnow(), ua.VariantType.DateTime))

def set_node_type(dtype_name, var):
    if dtype_name == 'DateTime':
        dv = ua.Variant(datetime.utcfromtimestamp(0.0), ua.VariantType.DateTime)
    else:
        return None
    return var.set_value(dv)

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def main(args):
    rosserver = ROSServer()

if __name__ == "__main__":
    main(sys.argv)
