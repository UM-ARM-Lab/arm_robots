import rospy
import tf2_ros as tf
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def block_publisher():

    tag_id = 1 # TODO: Generalize this
    block_size = np.array([0.145, 0.09, 0.051]) # (L x W x H) in meters
    tag_size = 0.09
    block_color = np.array([217, 95, 172, 255])/255 # (R, G, B, A)

    rospy.init_node('rsbox_publisher_node')
    tf_buffer = tf.BufferCore()
    transformer = tf.TransformListener(tf_buffer)
    tf_broadcaster = tf.TransformBroadcaster()
    publisher = rospy.Publisher('realsense_box', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)

    # import pdb; pdb.set_trace()
    rospy.sleep(1)
    tf_frames = tf_buffer._getFrameStrings()
    tag_frames = [tf_i for tf_i in tf_frames if 'tag_{}'.format(tag_id) in tf_i]

    # Table Marker
    markers = []
    for i, tag_i in enumerate(tag_frames):
        marker = Marker()
        marker.id = i
        marker.header.frame_id = tag_i
        marker.type = Marker.CUBE
        # size
        marker.scale.x = block_size[0]
        marker.scale.y = block_size[1]
        marker.scale.z = block_size[2]
        # pose (with respect to the frame 'med_table')
        marker.pose.position.x = 0.5 * block_size[0] - tag_size * 0.5
        marker.pose.position.y = 0
        marker.pose.position.z =  -0.5 * block_size[2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        # color
        marker.color.r = block_color[0]
        marker.color.g = block_color[1]
        marker.color.b = block_color[2]
        marker.color.a = block_color[3]
        markers.append(marker)

    # import pdb; pdb.set_trace()

    marker_array = MarkerArray()
    marker_array.markers = markers


    ts = []
    for i, marker in enumerate(markers):
        tag_i = tag_frames[i]
        t = TransformStamped()
        # print('child_id', tf_i['child_id'])
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = tag_i
        t.child_frame_id = 'object_center_{}'.format(i)
        t.transform.translation.x = 0.5 * block_size[0] - tag_size * 0.5
        t.transform.translation.y = 0
        t.transform.translation.z = -0.5 * block_size[2]
        q_i = quaternion_from_euler(-np.pi/2,0,0)
        t.transform.rotation.x = q_i[0]
        t.transform.rotation.y = q_i[1]
        t.transform.rotation.z = q_i[2]
        t.transform.rotation.w = q_i[3]
        ts.append(t)
    # import pdb; pdb.set_trace()


    while not rospy.is_shutdown():
        publisher.publish(marker_array)
        for t in ts:
            t.header.stamp = rospy.Time.now()
            tf_broadcaster.sendTransform(t)
        rate.sleep()


if __name__ == '__main__':
    try:
        block_publisher()
    except rospy.ROSInterruptException:
        pass
