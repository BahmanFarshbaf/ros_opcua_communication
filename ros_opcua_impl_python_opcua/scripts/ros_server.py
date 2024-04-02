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

        self.server.get_node('ns=2;i=6014').set_value(ua.Variant(1, ua.VariantType.UInt16))
        self.server.get_node('ns=2;i=6013').set_value(ua.Variant('1.0.0', ua.VariantType.String))
        self.server.get_node('ns=2;i=6012').set_value(ua.Variant('VetronSewingRobot', ua.VariantType.String))

        alarms = self.server.get_node('ns=2;i=6072')
        alarms.set_value(ua.Variant([0,0,0,0], ua.VariantType.UInt16))
        alarms.set_value_rank(1)
        alarms.set_array_dimensions([0])

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
    if dtype_name == 'Boolean':
        dv = ua.Variant(False, ua.VariantType.Boolean)
    elif dtype_name == 'DateTime':
        dv = ua.Variant(datetime.utcfromtimestamp(0.0), ua.VariantType.DateTime)
    elif dtype_name == 'Int16':
        dv = ua.Variant(0, ua.VariantType.Int16)
    elif dtype_name == 'UInt16':
        dv = ua.Variant(0, ua.VariantType.UInt16)
    elif dtype_name == 'Int32':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif dtype_name == 'UInt32':
        dv = ua.Variant(0, ua.VariantType.UInt32)
    elif dtype_name == 'Int64':
        dv = ua.Variant(0, ua.VariantType.Int64)
    elif dtype_name == 'UInt64':
        dv = ua.Variant(0, ua.VariantType.UInt64)
    elif dtype_name == 'Float' or dtype_name == 'Float32' or dtype_name == 'Float64':
        dv = ua.Variant(0.0, ua.VariantType.Float)
    elif dtype_name == 'String':
        dv = ua.Variant('', ua.VariantType.String)
    else:
        rospy.logerr("Can't create node with type" + str(dtype_name))
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
