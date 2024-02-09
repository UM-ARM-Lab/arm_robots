from arm_robots.victor import Victor
from victor_hardware_interfaces.msg import ControlMode, MotionStatus


def move_to_group_state(victor: Victor, joint_group: str, group_state: str):
    """
    Group name and state are strings that are defined by the robot's SRDF file (e.g. victor.srdf)
    """
    raise NotImplementedError()


def get_force_norm(status: MotionStatus):
    raise NotImplementedError()

# def get_median_filtered_left_force(victor):
#     status = victor.get_left_arm_status()
#     f = victor.get_force_norm(status)
#     victor.get_median_filtered_left_force.queue.append(f)
#     current_history = np.array(victor.get_median_filtered_left_force.queue)
#     filtered_force = signal.medfilt(current_history)[-1]
#     return filtered_force
#
# get_median_filtered_left_force.queue = collections.deque(maxlen=100)
#
# def get_median_filtered_right_force(victor):
#     status = victor.get_right_arm_status()
#     f = victor.get_force_norm(status)
#     victor.get_median_filtered_right_force.queue.append(f)
#     current_history = np.array(victor.get_median_filtered_right_force.queue)
#     filtered_force = signal.medfilt(current_history)[-1]
#     return filtered_force
#
# get_median_filtered_right_force.queue = collections.deque(maxlen=100)
#
# def stop_on_force_cb(victor, client, feedback):
#     rclpy.logwarn("wrong cb 1")
#     if victor.use_force_trigger:
#         status = victor.get_arms_statuses()
#         left_force = victor.get_force_norm(status['left'])
#         right_force = victor.get_force_norm(status['right'])
#         left_force_change = np.abs(left_force - victor.get_median_filtered_left_force())
#         right_force_change = np.abs(right_force - victor.get_median_filtered_right_force())
#
#         rclpy.logerr(f"{left_force_change} {right_force_change}")
#         left_force_change_msg = Float32()
#         left_force_change_msg.data = left_force_change
#         victor.left_force_change_sub.publish(left_force_change_msg)
#
#         rclpy.logerr(f"{right_force_change} {right_force_change}")
#         right_force_change_msg = Float32()
#         right_force_change_msg.data = right_force_change
#         victor.right_force_change_sub.publish(right_force_change_msg)
#
#         stop = left_force_change > victor.force_trigger or right_force_change > victor.force_trigger
#         if stop:
#             rclpy.logwarn("CANCELING!")
#             client.cancel_all_goals()
