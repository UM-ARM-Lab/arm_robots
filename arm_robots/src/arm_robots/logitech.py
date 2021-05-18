import numpy as np
from inputs import InputEvent

from arc_utilities.listener import Listener
from sensor_msgs.msg import Joy


class Logitech:

    def __init__(self, joystick_topic="joy"):
        self.xbox_listener = Listener(joystick_topic, Joy)

    def x_clicked(self, event: InputEvent):
        return event.ev_type == 'Key' and event.code == 'BTN_TRIGGER' and event.state == 0

    def get_axis_normalized(self, axis: int):
        joy_msg = self.xbox_listener.get()
        if axis == 0:
            return -joy_msg.axes[axis]
        elif axis == 1:
            return joy_msg.axes[axis]
        elif axis == 4:
            return -joy_msg.axes[axis]
        elif axis == 5:
            return joy_msg.axes[axis]
        else:
            raise NotImplementedError(f"axis {axis} is not implemented")

    def get_3d_delta(self):
        """
        We use the dpad for x,y and the left joystick to do z

        Returns:
            the [x, y, z], in the interval [-1.0, 1.0]

        """
        return np.array([self.get_axis_normalized(4),
                         self.get_axis_normalized(5),
                         self.get_axis_normalized(1)])
