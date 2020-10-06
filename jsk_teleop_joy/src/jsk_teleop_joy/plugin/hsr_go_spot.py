#!/usr/bin/env python

import math
import numpy

import rospy
import tf
import os
import time
import sys

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations

from jsk_rviz_plugins.msg import OverlayMenu
from status_history import StatusHistory
from jsk_hsr_startup import SpotMixin
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from jsk_teleop_joy.confirm_menu import YesNoMenu
from joy_rviz_view_controller import RVizViewController

def signed_square(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign

class HsrGoSpot(JSKJoyPlugin):
    '''
Usage:
Up/Down: choose from menu

circle/square/triangle: publish cooperating command

Args:
frame_id [String, default: base_footprint]: frame_id of publishing pose, overwritten by parameter ~frame_id
map_frame_id [String, default: map]: frame_id of map
title [String, default: SpotList]: title of the menu
#spot_array [String, default: spots_marker_array]: rostopic name to load spot array
base_pose [String, default: base_pose]: topic name for publishing base pose
z [float, default: 0.1]: initial value for z, overwritten by arg set_base
set_base [String, default: set_base]: topic name for publishing pose topic to set master
prefix [String, default: /map/]: prefix of the spots of interest. will be displayed as menu name
command [String, default: command]: topic name for publishing the command
triangle_cmd [String, default: GS_TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd [String, default: GS_CIRCLE_CMD]: command text when circle button is pressed
square_cmd [String, default: GS_SQUARE_CMD]: command text when square button is pressed
    '''
    STATE_INITIALIZATION = 1
    STATE_RUNNING = 2
    STATE_WAIT_FOR_JOY = 3

    MODE_MENU = 0
    MODE_MARKER = 1
    MODE_DELETE = 2
    MODE_SAVE = 3
    mode = 0

    spot_names = []

    def __init__(self, name, args):
        JSKJoyPlugin.__init__(self, name, args)
        self.current_item_index = 0
        self.selecting_item_index = 0

        self.title = self.getArg('title', 'SpotList')
        self.publish_pose = self.getArg('publish_pose', True)
        self.pose_pub = rospy.Publisher(self.getArg('base_pose', 'base_pose'),
                                        PoseStamped, queue_size=10)
        self.target_pub = rospy.Publisher(self.getArg('target_pose', 'target_pose'),
                                        PoseStamped, queue_size=1)
        self.z = self.getArg('z', 0.1)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.position.z = self.z

        self.prefix = self.getArg('prefix', '/map/')
        menu_topic = self.getArg('menu', 'dynamic_menu')
        self.menu_pub = rospy.Publisher(menu_topic, OverlayMenu, queue_size=10)
#        self.spot_topic = self.getArg('spot_array', 'spots_marker_array')
#        self.init_spot_marker_array()

        self.pose_sub = rospy.Subscriber(self.getArg('set_base', 'set_base'),
                                        PoseStamped, self.set_pose_cb, queue_size=1)

        self.frame_id = self.getArg('frame_id', 'base_footprint')
        self.map_frame_id = self.getArg('map_frame_id', 'map')
        self.tf_listener = tf.TransformListener()
        self.prev_time = rospy.Time.from_sec(time.time())
        self.mixin = SpotMixin()

        #self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
        #                                    String, queue_size=1)
        #self.triangle_cmd = self.getArg('triangle_cmd', 'GS_TRIANGLE_CMD')
        #self.square_cmd = self.getArg('square_cmd', 'GS_SQUARE_CMD')
        #self.circle_cmd = self.getArg('circle_cmd', 'GS_CIRCLE_CMD')

        delete_cap = "Delete current spot? (This cannot be undone)"
        self.delete_menu = YesNoMenu(menu_topic, delete_cap, self.delete_yes, self.delete_no)
        save_cap = "Save current spot?"
        self.save_menu = YesNoMenu(menu_topic, save_cap, self.save_yes, self.save_no)

        self.init_pre_pose()
        self.view_controller = RVizViewController(name, args)
        self.view_controller.pre_pose = self.pre_pose

        self.start()

    def init_spot_marker_array(self):
        self.spot_marker_array = []
        spot_msg = MarkerArray()
        try:
            spot_msg = rospy.wait_for_message(self.spot_topic, MarkerArray, 20.0)
        except rospy.exceptions.ROSException:
            rospy.logerr("Cannot get spot array from \"/{}\"".format(self.spot_topic))
        index = []
        for marker in spot_msg.markers:
            if marker.ns == "pin_label" and marker.text.startswith(self.prefix):
                item = [0] * 3
                item[0] = marker.id
                item[1] = marker.text[len(self.prefix):]
                item[2] = marker.pose
                index.append(item)
        if len(index) == 0:
            rospy.logwarn('no valid spots are loaded')
        else:
            diff_q = tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0)
            i = 0
            for marker in spot_msg.markers:
                if marker.ns == "pin_head" and marker.id == index[i][0]:
                    item = [0] * 2
                    item[0] = index[i][1]
                    item[1] = index[i][2]
                    #item[1].orientation = marker.pose.orientation
                    q = numpy.array((marker.pose.orientation.x,
                                     marker.pose.orientation.y,
                                     marker.pose.orientation.z,
                                     marker.pose.orientation.w))
                    new_q = tf.transformations.quaternion_multiply(q, diff_q)
                    item[1].orientation.x = new_q[0]
                    item[1].orientation.y = new_q[1]
                    item[1].orientation.z = new_q[2]
                    item[1].orientation.w = new_q[3]
                    self.spot_marker_array.append(item)
                    i += 1
                if i >= len(index):
                    break

    def init_pre_pose(self):
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.pre_pose.pose.position.z = self.z
        if rospy.has_param('~frame_id'):
            self.frame_id = rospy.get_param('~frame_id')
        self.pre_pose.header.frame_id = self.frame_id
        a = self.pre_pose
        a.header.stamp = rospy.Time(0)
        while not rospy.is_shutdown():
            ret = self.tf_listener.waitForTransform(self.map_frame_id, a.header.frame_id,
                                                    rospy.Time(0), rospy.Duration(3.0))
            print(ret)
            try:
                self.pre_pose = self.tf_listener.transformPose(self.map_frame_id, a)
            except Exception as e:
                print(e)
                continue
            break

    def set_pose_cb(self, pose):
        pose.header.stamp = rospy.Time(0)
        self.pre_pose = self.tf_listener.transformPose(self.map_frame_id, pose)
        if self.publish_pose:
            self.pose_pub.publish(self.pre_pose)

    def load_items(self):
        self.spot_names = []
        spot_list = self.mixin.get_spots()
        rospy.loginfo("Get " + str(len(spot_list)) + "spots")
        for spot in spot_list:
            if spot.startswith(self.prefix):
                self.spot_names.append(spot[len(self.prefix):])

    def switch_item(self, index):
        self.current_item_index = index
        if len(self.spot_names) <= self.current_item_index:
            self.current_item_index = 0
        elif self.current_item_index < 0:
            self.current_item_index = len(self.spot_names)
        self.current_item = self.spot_names[self.current_item_index]
        self.display(self.current_item)

    def start(self):
        self.load_items()
        self.publish_menu(0, close=True) # close menu anyway
        if len(self.spot_names) == 0:
            rospy.logwarn('no valid spots are loaded')
            self.mode = self.MODE_MARKER
            return True
        self.current_item = self.spot_names[0]
        return True

    def publish_menu(self, index, close=False):
        menu = OverlayMenu()
        menu.menus = self.spot_names
        menu.current_index = index
        menu.title = self.prefix
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
        self.menu_pub.publish(menu)

    def joyCB(self, status, history):
        if history.length() > 0:
            latest = history.latest()
        if self.mode == self.MODE_MENU and len(self.spot_names) > 0:
            if history.new(status, "down") or history.new(status, "left_analog_down"):
                self.selecting_item_index = self.selecting_item_index + 1
                if self.selecting_item_index >= len(self.spot_names):
                    self.selecting_item_index = 0
                self.publish_menu(self.selecting_item_index)
                self.switch_item(self.selecting_item_index)
            elif history.new(status, "up") or history.new(status, "left_analog_up"):
                self.selecting_item_index = self.selecting_item_index - 1
                if self.selecting_item_index < 0:
                    self.selecting_item_index = len(self.spot_names) - 1
                self.publish_menu(self.selecting_item_index)
                self.switch_item(self.selecting_item_index)
            elif status.circle and not latest.circle:
                #TODO circle edit pose
                self.publish_menu(self.selecting_item_index, close=True)
                self.mode = self.MODE_MARKER
                rospy.loginfo("edit mode")
            elif status.triangle and not latest.triangle:
                #TODO triangle go_spot
                self.mixin.go_to_spot(self.prefix + self.current_item)
                rospy.loginfo("Go to spot: " + self.prefix + self.current_item)
            elif status.square and not latest.square:
                #TODO square delete_spot
                self.publish_menu(self.selecting_item_index, close=True)
                self.delete_menu.publish_menu(1)
                self.mode = self.MODE_DELETE
                rospy.loginfo("delete mode")
            #elif status.cross and not latest.cross:
            #    
            else:
                self.publish_menu(self.selecting_item_index)
        elif self.mode == self.MODE_MARKER:
            self.marker_joy_cb(status, history)
        elif self.mode == self.MODE_DELETE:
            self.delete_menu.joy_cb(status, history)
        elif self.mode == self.MODE_SAVE:
            self.save_menu.joy_cb(status, history)

    def display(self, item):
        fullname = self.prefix + item
        rospy.loginfo("Load spot: " + str(fullname))
        pose = self.pre_pose
        pose.header.frame_id = self.map_frame_id
        pose.header.stamp = rospy.Time(0.0)
        pose.pose = self.mixin.lookup_spot(fullname).pose
        pose.pose.position.z = self.z
        self.pose_pub.publish(pose)

    def marker_joy_cb(self, status, history):
        pre_pose = self.pre_pose
        if history.length() > 0:
            latest = history.latest()
            if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
                self.view_controller.followView(not self.followView())
        if self.view_controller.control_view:
            self.view_controller.joyCB(status, history)
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.map_frame_id
        new_pose.header.stamp = rospy.Time(0.0)
        # move in local
        if not status.R3:
            # xy
            if status.square:
                scale = 10.0
            else:
                dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
                if dist > 0.9:
                    scale = 20.0
                else:
                    scale = 60.0
            x_diff = signed_square(status.left_analog_y) / scale
            y_diff = signed_square(status.left_analog_x) / scale
            # No translation in z
            local_move = numpy.array((x_diff, y_diff, 0.0, 1.0))
        else:
            local_move = numpy.array((0.0, 0.0, 0.0, 1.0))
        q = numpy.array((pre_pose.pose.orientation.x,
                        pre_pose.pose.orientation.y,
                        pre_pose.pose.orientation.z,
                        pre_pose.pose.orientation.w))
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q), local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = self.z
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.02
        if not status.R3:
            if status.L1:
                if status.square:
                    yaw = yaw + DTHETA * 5
                elif history.all(lambda s: s.L1):
                    yaw = yaw + DTHETA * 2
                else:
                    yaw = yaw + DTHETA
            elif status.R1:
                if status.square:
                    yaw = yaw - DTHETA * 5
                elif history.all(lambda s: s.R1):
                    yaw = yaw - DTHETA * 2
                else:
                    yaw = yaw - DTHETA
            # No rotation in pitch and roll
        diff_q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        new_q = tf.transformations.quaternion_multiply(q, diff_q)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]
        if not (status.R3 and status.R2 and status.L2):
            if status.circle and not latest.circle:
                self.target_pub.publish(new_pose)
            if status.triangle and not latest.triangle:
                self.init_pre_pose()
                new_pose = self.pre_pose
                self.save_menu.publish_menu(1)
                self.mode = self.MODE_SAVE
            if status.cross and not latest.cross:
#                self.init_spot_marker_array()
                self.load_items()
                self.publish_menu(self.selecting_item_index)
                self.mode = self.MODE_MENU

        # publish at 10hz
        if self.publish_pose:
            now = rospy.Time.from_sec(time.time())
            # placement.time_from_start = now - self.prev_time
            if (now - self.prev_time).to_sec() > 1 / 30.0:
                self.pose_pub.publish(new_pose)
                self.prev_time = now

        self.pre_pose = new_pose

    def save_yes(self):
        name = "teleop-temp"
        fullname = self.prefix + name
        self.mixin.save_current_spot(fullname)
        rospy.loginfo("Save as: " + fullname)
        self.mode = self.MODE_MARKER
        self.publish_help()

    def save_no(self):
        self.mode = self.MODE_MARKER
        self.publish_help()

    def delete_yes(self):
        self.mixin.delete_spot(self.prefix + self.current_item)
#        self.init_spot_marker_array()
        self.load_items()
        self.mode = self.MODE_MENU
        self.publish_menu(0)
        self.publish_help()

    def delete_no(self):
        self.mode = self.MODE_MENU
        self.publish_menu(self.current_item_index)
        self.publish_help()

    def publish_help(self):
        return

    def enable(self):
        self.start()
        # TODO save the previous pose?
        self.selecting_item_index = 0
        self.switch_item(0)
        self.publish_menu(self.selecting_item_index, close=False)

    def disable(self):
        self.publish_menu(self.selecting_item_index, close=True)
        # TODO Back to main menu

def main():
    global g_manager
    rospy.sleep(1)
    rospy.init_node('jsk_teleop_joy')
    g_manager = JoyManager()
    result = g_manager.start()
    if not result:
        rospy.logfatal("Fatal Error")
        return False
    else:
        rospy.spin()

if __name__ == '__main__':
    main()
