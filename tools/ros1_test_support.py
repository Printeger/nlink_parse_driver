"""ROS1 test resource ownership and wall-time waits (also when /clock stops)."""
import time
import rospy
import rosgraph

class TestNode:
    def __init__(self, name):
        if not rospy.core.is_initialized():
            rospy.init_node(name, anonymous=True, disable_signals=True)
        self.resources = []

    def subscribe(self, message_type, topic, callback, depth=100):
        sub = rospy.Subscriber(topic, message_type, callback, queue_size=depth,
                               tcp_nodelay=True)
        self.resources.append(sub)
        return sub

    def advertise(self, message_type, topic, depth=100):
        pub = rospy.Publisher(topic, message_type, queue_size=depth, latch=False)
        self.resources.append(pub)
        return pub

    def close(self):
        for resource in self.resources:
            resource.unregister()
        self.resources.clear()

    def publishers(self):
        return dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])

    def count_publishers(self, topic):
        return len(self.publishers().get(topic, []))


def spin_wall(node, timeout_sec=.01):
    time.sleep(max(0, timeout_sec))
