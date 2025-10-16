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
        self.namespace_ros = rospy.get_param("/opcua/namespace")
        rospy.init_node("opcua")
        self.server = Server()
        
        with open('/data/workcell_smp_irb2600/config/irs_opcua_server.txt', 'r') as txt:
            txtfile = txt.read()
            
        self.server.set_endpoint(txtfile)
        # NEW: Load the XML file with the new string-based NodeIds
        self.server.import_xml("/data/workcell_smp_irb2600/config/irs_opcua_nodes_string_ids.xml")
        self.server.start()
        
        # NEW: Get the namespace index dynamically. It's better than hardcoding 'ns=2'.
        self.idx = self.server.get_namespace_index(self.namespace_ros)
        print(f"OPC UA Server using namespace index: {self.idx}")

        handler = SubHandler()
        sub = self.server.create_subscription(100, handler)

        # --- NEW: Browse for all variable nodes instead of looping a numeric range ---
        
        # 1. Get the root node from which to start browsing
        root_node = self.server.get_node(f"ns={self.idx};s=IRS_MES")
        
        # 2. Recursively find all variable nodes under the root
        all_variable_nodes = []
        self.browse_and_collect_vars(root_node, all_variable_nodes)
        print(f"Found {len(all_variable_nodes)} total variable nodes.")

        # 3. Define the nodes to exclude by their full string ID
        nodes_to_exclude = [
            f"ns={self.idx};s=IRS_MES.System.Client.SystemTimeUTC",
            f"ns={self.idx};s=IRS_MES.System.Server.SystemTimeUTC"
        ]

        # 4. Create the final list by filtering out the excluded nodes
        nodelist = [
            node for node in all_variable_nodes 
            if node.nodeid.to_string() not in nodes_to_exclude
        ]
        
        # 5. Make all nodes in the final list writable
        for var in nodelist:
            var.set_writable()

        print(f"Subscribing to data changes on {len(nodelist)} nodes after exclusions.")
        sub.subscribe_data_change(nodelist)
        
        # --- End of new node discovery logic ---
        
        # NEW: Get the server time node and pass it directly to the timer function
        server_time_node = self.server.get_node(f"ns={self.idx};s=IRS_MES.System.Server.SystemTimeUTC")
        rt = RepeatedTimer(30, timeupdater, server_time_node)

        print("ROS OPCUA Server initialized.")
        while not rospy.is_shutdown():
            rospy.spin()
             
        time.sleep(0)  # used to be 60

        self.server.stop()
        rt.stop()
        print("ROS OPCUA Server stopped.")
        quit()

    # NEW: A recursive function to browse the node tree and collect all variables
    def browse_and_collect_vars(self, parent_node, collection_list):
        """
        Recursively browses from a parent node, finds all child nodes of type
        UAVariable, and adds them to the collection_list.
        """
        for child_node in parent_node.get_children():
            # If the child is a variable, add it to our list
            if child_node.get_node_class() == ua.NodeClass.Variable:
                collection_list.append(child_node)
            # Recurse into the child to find its children, regardless of its type
            self.browse_and_collect_vars(child_node, collection_list)


# NEW: The timeupdater function now accepts the node object directly
def timeupdater(node_to_update):
    """Updates the timestamp on the given server time node."""
    node_to_update.set_value(ua.Variant(datetime.utcnow(), ua.VariantType.DateTime))

# This function remains unchanged
def set_node_type(dtype_name, var):
    if dtype_name == 'DateTime':
        dv = ua.Variant(datetime.utcfromtimestamp(0.0), ua.VariantType.DateTime)
    else:
        return None
    return var.set_value(dv)

# This class remains unchanged
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
