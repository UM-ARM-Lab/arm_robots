import roscpp_initializer

import rospy


def rospy_and_cpp_init(name):
    roscpp_initializer.init_node("cpp_" + name, [], disable_signals=True)
    rospy.init_node(name)