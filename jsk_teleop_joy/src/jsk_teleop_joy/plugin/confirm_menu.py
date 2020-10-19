import rospy

from jsk_rviz_plugins.msg import OverlayMenu
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

class YesNoMenu(JSKJoyPlugin):
    def __init__(self, topic, cap, func_y, func_n):
        self.cap = cap
        self.func_yes = func_y
        self.func_no = func_n
        self.index = 0
        self.menu_pub = rospy.Publisher(topic, OverlayMenu, queue_size=1)

    def joy_cb(self, status, history):
        if history.new(status, "down") or history.new(status, "left_analog_down") \
            or history.new(status, "up") or history.new(status, "left_analog_up"):
            self.index = 1 - self.index
            self.publish_menu(self.index)
        elif history.new(status, "circle") and self.index == 0:
            self.func_yes()
            self.publish_menu(self.index, close=True)
            return True
        elif history.new(status, "cross") \
            or (history.new(status, "circle") and self.index == 1):
            self.func_no()
            self.publish_menu(self.index, close=True)
            return False

    def publish_menu(self, index, close=False):
        self.index = index
        menu = OverlayMenu()
        menu.menus = ["YES", "NO"]
        menu.current_index = index
        menu.title = self.cap
        self.isClosed = False
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
            self.isClosed = True
        self.menu_pub.publish(menu)
