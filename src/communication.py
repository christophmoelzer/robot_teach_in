from pydoc import cli
from unicodedata import digit
import rospy
import math
import time
from opcua import Client, ua
from std_msgs.msg import String

# http://docs.ros.org/en/kinetic/api/ros_opcua_impl_python_opcua/html/client-example_8py_source.html

class SubHandler(object):
    
    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another 
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        print("Python: New data change event", node, val)
        print(val)

    def event_notification(self, event):
        print("Python: New event", event)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "data %s", data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "foo %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

class PLCConnection():

    client = None
    root = None
    sub = None
    handle = None

    def init(self):
        self.client = Client("opc.tcp://192.168.1.156:4840")
        self.client.connect()
        self.client.load_type_definitions()
        self.root = self.client.get_root_node()  
        
        
    def subscribe(self):
        myvar = self.root.get_child(["0:Objects", "3:PLC_2", "3:DataBlocksGlobal", "3:OPC_Communication", "3:xLiveSign_out"])
        handler = SubHandler()
        self.sub = self.client.create_subscription(500, handler)
        self.handle = self.sub.subscribe_data_change(myvar)

    def call_method(self):
        obj = self.root.get_child(["0:Objects", "3:PLC_2", "3:DataBlocksInstance", "3:OPC_UA_ROS_DB"])
        sys = self.client.get_node('ns=3;s="OPC_UA_ROS_DB"')
        print('sys = ', sys)
        print('obj = ', obj)
        method = self.client.get_node('ns=3;s="OPC_UA_ROS_DB".Method')
        print('method = ', method)
        
        res = sys.call_method(method, ua.Variant(1, ua.VariantType.Int16), ua.Variant(4, ua.VariantType.Int16), ua.Variant(False, ua.VariantType.Boolean), ua.Variant(True, ua.VariantType.Boolean))
        #res = obj.call_method("3:OPC_UA_ROS")
        print('res = ', res)
    
    def disconnect(self):
        self.sub.unsubscribe(self.handle)
        self.sub.delete()
        self.client.disconnect()


if __name__ == '__main__':
    try:
        plc = PLCConnection()
        plc.init()
        #plc.subscribe()
        plc.call_method()
    finally:
        #time.sleep(10)
        #plc.disconnect()
        pass