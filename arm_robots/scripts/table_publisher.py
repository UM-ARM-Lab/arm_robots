import rospy
import tf
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray





def table_publisher():
    pub = rospy.Publisher('med_table', Marker, queue_size=100)
    rospy.init_node('table_publisher_node', anonymous=True)
    rate = rospy.Rate(10)

    table_size = np.array([1.53, 0.92, 0.76]) # (L x W x H) in meters
    table_color = np.array([224, 194, 112, 255])/255 # (R, G, B, A)
    plate_size = 0.305
    # Table Marker
    marker = Marker()
    marker.header.frame_id = 'med_base'
    marker.type = Marker.CUBE
    # size
    marker.scale.x = table_size[0]
    marker.scale.y = table_size[1]
    marker.scale.z = table_size[2]
    # pose (with respect to the frame 'med_table')
    marker.pose.position.x = table_size[0]/2 - plate_size*0.5
    marker.pose.position.y = -table_size[1]/2 +0.535 - plate_size*0.5
    marker.pose.position.z = -table_size[2]/2
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    # color
    marker.color.r = table_color[0]
    marker.color.g = table_color[1]
    marker.color.b = table_color[2]
    marker.color.a = table_color[3]

    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()


if __name__ == '__main__':
    try:
        table_publisher()
    except rospy.ROSInterruptException:
        pass
