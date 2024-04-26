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

def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)

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
        self.server.import_xml("/data/workcell_smp_irb2600/config/irs_opcua_nodes.xml")
        self.server.start()

        handler = SubHandler()
        sub = self.server.create_subscription(100, handler)

        varnodeids = range(6001, 6090)
        nodelist = []
        try:
            for nd in varnodeids:
                if nd not in range(6005, 6012):
                    var = self.server.get_node('ns=2;i=' + str(nd))
                    nodelist.append(var)
                dtype = var.get_data_type()
                if dtype.NamespaceIndex == 0 and dtype.Identifier in o_ids.ObjectIdNames:
                    dtype_name = o_ids.ObjectIdNames[dtype.Identifier]
                    set_node_type(dtype_name, var)
                else:
                    dtype_name = dtype.to_string()
                    rank = var.get_value_rank()
                self.server.get_node('ns=2;i=' + str(nd)).set_writable()
            del nodelist[3]
            del nodelist[6]
            rt = RepeatedTimer(30, timeupdater, self.server)
        except:
            None

        sub.subscribe_data_change(nodelist)

        print("ROS OPCUA Server initialized.")
        while not rospy.is_shutdown():
            rospy.spin()
            
        time.sleep(0)  # used to be 60

        self.server.stop()
        rt.stop()
        print("ROS OPCUA Server stopped.")
        quit()

def timeupdater(serverself):
    serverself.get_node(ua.NodeId.from_string('ns=2;i=6015')).set_value(ua.Variant(datetime.utcnow(), ua.VariantType.DateTime))

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
