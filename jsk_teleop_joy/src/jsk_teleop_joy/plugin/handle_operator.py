#!/usr/bin/env python

import math
import numpy
import copy

import rospy
import os
import sys
import time

import imp
try:
    imp.find_module("sensor_msgs")
except:
    import roslib; roslib.load_manifest('jsk_teleop_joy')

from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA, Float32
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import tf.transformations

from jsk_recognition_msgs.msg import BoundingBox
from jsk_rviz_plugins.msg import OverlayMenu, OverlayText
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from joy_rviz_view_controller import RVizViewController

def signed_square(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign

class HandleOperator(JSKJoyPlugin):
    '''
Usage:
Check publish_help() for controller configurations.

Args:
publish_pose [Boolean, default: True]: publish pose or not
title [String, default: MarkerList]: title of the menu
history [String, default: history]: rosparam name to load pose list
marker_pose [String, default: marker_pose]: topic name to publishing marker pose
menu [String, default: dynamic_menu]: topic name to publish the menu
help_text [String, default: help_text]: topic name to publish overlay help text
lock_xy [Boolean, default: False]: to keep x,y scale equal or not
frame_id [String, default: map]: frame_id of publishing pose
namespace [String, default: joy_markers]: namespace to publish the marker
type [Int, default: 3]: type of the marker, cylinder by default
show_label [Boolean, default: False]: display labels for marks or not
    '''
    STATE_INITIALIZATION = 1
    STATE_RUNNING = 2
    STATE_WAIT_FOR_JOY = 3

    MODE_MENU = 0
    MODE_HANDLE = 1
    MODE_MARKER = 2
    MODE_AXIS = 3
    MODE_OPERATE = 4
    MODE_DELETE = 5
    MODE_UNSAVED = 6

    menu_list = ['Set Handle', 'Set Door Surface', 'Set Door Axis', 'Operate']

    def __init__(self, name, args):
        JSKJoyPlugin.__init__(self, name, args)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.prev_time = rospy.Time.from_sec(time.time())
        self.publish_pose = self.getArg('publish_pose', True)
        self.view_controller = RVizViewController(name, args)
        self.view_controller.pre_pose = self.pre_pose
        self.current_index = 0
        self.confirm_index = 0
        self.selecting_index = 0
        self.markers = MarkerArray()
        self.title = self.getArg('title', 'MarkerList')
        self.pose_pub = rospy.Publisher(self.getArg('marker_pose', 'marker_pose'),
                                        PoseStamped, queue_size=10)
        self.menu_pub = rospy.Publisher(self.getArg('menu', 'dynamic_menu'),
                                        OverlayMenu, queue_size=10)
        self.marker_array_pub = rospy.Publisher(self.getArg('marker_array', 'marker_array'),
                                                MarkerArray, queue_size=10)
        self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                           String, queue_size=1)
        self.marker_sub = rospy.Subscriber(self.getArg('set_marker', 'set_marker'),
                                           BoundingBox, self.set_marker_cb, queue_size=1)
        self.help_pub = rospy.Publisher(self.getArg('help_text', 'help_text'),
                                        OverlayText, queue_size=1)
        self.frame_id = self.getArg('frame_id', 'map')
        self.lock_xy = self.getArg('lock_xy', False)
        self.type = self.getArg('type', 3)
        self.namespace = self.getArg('namespace', 'joy_markers')
        self.show_label = self.getArg('show_label', False)
        self.isClosed = True

        self.mode = 0

        #TODO self.loadMarkers()
        self.start()

    def start(self):
        self.publish_menu(0, close=True) # close menu anyway
        if len(self.markers.markers) == 0:
            rospy.loginfo('The marker array is now empty')
            self.current_marker = None
        else:
            self.current_marker = self.markers.markers[0]
        return True

    def publish_menu(self, index, close=False):
        menu = OverlayMenu()
        menu.menus = self.menu_list
        menu.current_index = index
        menu.title = self.title
        self.isClosed = False
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
            self.isClosed = True
        self.menu_pub.publish(menu)

    def joyCB(self, status, history):
        if history.length() > 0:
            latest = history.latest()
        if self.mode == self.MODE_MENU:
            if status.triangle:
                # move/switch order
                if history.new(status, "R1"):
                    #TODO preview ik
                    rospy.logdebug("preview ik")
            else:
                if history.new(status, "down") or history.new(status, "left_analog_down"):
                    self.selecting_index = self.selecting_index + 1
                    if self.selecting_index >= len(self.menu_list):
                        self.selecting_index = 0
                    self.publish_menu(self.selecting_index)
                    self.switch_marker(self.selecting_index)
                elif history.new(status, "up") or history.new(status, "left_analog_up"):
                    self.selecting_index = self.selecting_index - 1
                    if self.selecting_index < 0:
                        self.selecting_index = len(self.menu_list) - 1
                    self.publish_menu(self.selecting_index)
                    self.switch_marker(self.selecting_index)
                elif history.new(status, "circle"):
                    # close menu and edit
                    self.publish_menu(self.selecting_index, close=True)
                    if self.selecting_index == 3:
                        self.mode = self.MODE_OPERATE
                    else:
                        self.init_marker()
                        self.mode = self.MODE_MARKER
                elif history.new(status, "square"):
                    # delete current marker
                    if not self.current_marker == None:
                        self.publish_menu(self.selecting_index, close=True)
                        self.confirm_index = 0
                        self.publish_delete_menu(self.confirm_index)
                        self.mode = self.MODE_DELETE
                elif history.new(status, "start"):
                    #TODO execute?
                    rospy.logdebug("start pressed")
                else:
                    self.publish_menu(self.selecting_index)
                self.publish_help()
        elif self.mode == self.MODE_MARKER:
            self.marker_joy_cb(status, history)
        elif self.mode == self.MODE_DELETE:
            if history.new(status, "down") or history.new(status, "left_analog_down") \
                or history.new(status, "up") or history.new(status, "left_analog_up"):
                self.confirm_index = 1 - self.confirm_index
                self.publish_delete_menu(self.confirm_index)
            elif history.new(status, "circle") and self.confirm_index == 1:
                self.publish_delete_menu(self.confirm_index, close=True)
                self.delete_marker(self.current_index)
                self.mode = self.MODE_MENU
                self.publish_help()
                self.switch_marker(self.selecting_index)
                self.publish_markers()
            elif history.new(status, "cross") \
                or (history.new(status, "circle") and self.confirm_index == 0):
                self.publish_unsaved_menu(self.confirm_index, close=True)
                self.mode = self.MODE_MENU
                self.publish_help()
        elif self.mode == self.MODE_UNSAVED:
            if history.new(status, "down") or history.new(status, "left_analog_down") \
                or history.new(status, "up") or history.new(status, "left_analog_up"):
                self.confirm_index = 1 - self.confirm_index
                self.publish_unsaved_menu(self.confirm_index)
            elif history.new(status, "circle") and self.confirm_index == 1:
                self.publish_unsaved_menu(self.confirm_index, close=True)
                if self.pre_marker == None:
                    self.delete_marker(self.current_index)
                    self.current_marker = None
                    self.current_index -= 1
                else:
                    # restore pre_marker
                    self.current_marker = self.pre_marker
                    self.markers.markers[self.current_index] = self.current_marker
                    self.set_color(self.current_marker, highlight=False)
                self.mode = self.MODE_MENU
                self.publish_help()
                self.switch_marker(self.selecting_index)
                self.publish_markers()
            elif history.new(status, "cross") \
                or (history.new(status, "circle") and self.confirm_index == 0):
                self.publish_unsaved_menu(self.confirm_index, close=True)
                self.mode = self.MODE_MARKER
                self.publish_help()
        elif self.mode == self.MODE_OPERATE:
            if history.new(status, "cross"):
                self.mode = self.MODE_MENU
                self.publish_menu(self.current_index)
            elif status.circle:
                if status.left_analog_down:
                    self.command_pub.publish("PULLTEST+")
                if status.left_analog_up:
                    self.command_pub.publish("PULLTEST-")


    def switch_marker(self, index):
        self.current_index = index
        ns = ''
        rospy.loginfo("current index: " + str(index))
        if index == 0:
            ns = 'handle'
        elif index == 2:
            ns = 'axis'
        rospy.loginfo("current ns: " + ns)

        if not self.current_marker == None:
            self.set_color(self.current_marker, highlight=False)

        self.current_marker = None
        for m in self.markers.markers:
            if m.ns == ns:
                self.current_marker = m

        self.publish_markers()

    def delete_marker(self, index):
        rospy.logdebug("delete marker " + str(self.current_index))
        self.markers.markers[index].action = Marker.DELETE
        self.publish_markers()
        self.markers.markers.remove(self.markers.markers[index])

    def set_color(self, marker, highlight=False):
        if highlight:
            marker.color.r = 0.0
            marker.color.g = 1.0
        else:
            marker.color.r = 0.6
            marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    def publish_delete_menu(self, index, close=False):
        menu = OverlayMenu()
        menu.menus = ["No", "Yes"]
        menu.current_index = index
        menu.title = "Delete current marker? (Cannot undo)"
        self.isClosed = False
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
            self.isClosed = True
        self.menu_pub.publish(menu)

    def publish_unsaved_menu(self, index, close=False):
        menu = OverlayMenu()
        menu.menus = ["No", "Yes"]
        menu.current_index = index
        menu.title = "Quit edit mode? (Unsaved markers will be aborted)"
        self.isClosed = False
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
            self.isClosed = True
        self.menu_pub.publish(menu)

    def marker_joy_cb(self, status, history):
        pre_pose = self.pre_pose
        marker = self.current_marker
        if history.length() > 0:
            latest = history.latest()
            if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
                self.view_controller.followView(not view_controller.followView())
        if self.view_controller.control_view:
            self.view_controller.joyCB(status, history)
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)
        # translate in local
        if not status.R3 and not status.L1:
            # xy
            if status.square:
                scale = 20.0
            else:
                dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
                if dist > 0.9:
                    scale = 50.0
                else:
                    scale = 80.0
            x_diff = signed_square(status.left_analog_y) / scale
            y_diff = signed_square(status.left_analog_x) / scale
            # z
            if status.L2:
                z_diff = 0.005
            elif status.R2:
                z_diff = -0.005
            else:
                z_diff = 0.0
            if status.square:
                z_scale = 5.0
            elif history.all(lambda s: s.L2) or history.all(lambda s: s.R2):
                z_scale = 2.0
            else:
                z_scale = 1.0
            local_move = numpy.array((x_diff, y_diff,
                                      z_diff * z_scale, 
                                      1.0))
        else:
            local_move = numpy.array((0.0, 0.0, 0.0, 1.0))
        q = numpy.array((pre_pose.pose.orientation.x,
                                         pre_pose.pose.orientation.y,
                                         pre_pose.pose.orientation.z,
                                         pre_pose.pose.orientation.w))
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q), local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.02
        DSCALE = 0.002
        if not status.R3:
            # scale
            if status.L1:
                # scale xy
                if status.square:
                    xscale_diff = status.left_analog_y * DSCALE * 15
                    yscale_diff = status.left_analog_x * DSCALE * 15
                else:
                    xscale_diff = status.left_analog_y * DSCALE * 5
                    yscale_diff = status.left_analog_x * DSCALE * 5
                marker.scale.x = marker.scale.x + xscale_diff
                if self.lock_xy:
                    marker.scale.y = marker.scale.x
                marker.scale.y = marker.scale.y + yscale_diff
                if self.lock_xy:
                    marker.scale.x = marker.scale.y
                # scale z
                if status.up:
                    if status.square:
                        marker.scale.z = marker.scale.z + DSCALE * 5
                    elif history.all(lambda s: s.up):
                        marker.scale.z = marker.scale.z + DSCALE * 2
                    else:
                        marker.scale.z = marker.scale.z + DSCALE
                elif status.down:
                    if status.square:
                        marker.scale.z = marker.scale.z - DSCALE * 5
                    elif history.all(lambda s: s.down):
                        marker.scale.z = marker.scale.z - DSCALE * 2
                    else:
                        marker.scale.z = marker.scale.z - DSCALE
                if history.new(status, "R1"):
                    self.lock_xy = not self.lock_xy
                    if self.lock_xy:
                        marker.scale.x = marker.scale.y
                        # torque between cube (type=1) and cylinder (type=3)
                        self.type = 4 - self.type
                        for m in self.markers.markers:
                            m.type = self.type
                    else:
                        marker.scale.x *= 0.6

                # keep minimun scale value
                if not marker.scale.x > 0:
                    marker.scale.x = DSCALE
                if not marker.scale.y > 0:
                    marker.scale.y = DSCALE
                if not marker.scale.z > 0:
                    marker.scale.z = DSCALE

                ## uniform all markers' size
                #for m in self.markers.markers:
                #    m.scale = marker.scale
            # rotate
            elif status.R1:
                if status.left:
                    if status.square:
                        yaw = yaw + DTHETA * 5
                    elif history.all(lambda s: s.L1):
                        yaw = yaw + DTHETA * 2
                    else:
                        yaw = yaw + DTHETA
                elif status.right:
                    if status.square:
                        yaw = yaw - DTHETA * 5
                    elif history.all(lambda s: s.R1):
                        yaw = yaw - DTHETA * 2
                    else:
                        yaw = yaw - DTHETA
            else:
                if status.up:
                    if status.square:
                        pitch = pitch + DTHETA * 5
                    elif history.all(lambda s: s.up):
                        pitch = pitch + DTHETA * 2
                    else:
                        pitch = pitch + DTHETA
                elif status.down:
                    if status.square:
                        pitch = pitch - DTHETA * 5
                    elif history.all(lambda s: s.down):
                        pitch = pitch - DTHETA * 2
                    else:
                        pitch = pitch - DTHETA
                if status.right:
                    if status.square:
                        roll = roll + DTHETA * 5
                    elif history.all(lambda s: s.right):
                        roll = roll + DTHETA * 2
                    else:
                        roll = roll + DTHETA
                elif status.left:
                    if status.square:
                        roll = roll - DTHETA * 5
                    elif history.all(lambda s: s.left):
                        roll = roll - DTHETA * 2
                    else:
                        roll = roll - DTHETA
        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        new_q = tf.transformations.quaternion_multiply(q, diff_q)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]
        marker.pose = new_pose.pose

        # publish marker(s) at 10hz
        now = rospy.Time.from_sec(time.time())
        # placement.time_from_start = now - self.prev_time
        if (now - self.prev_time).to_sec() > 1 / 10.0:
            self.publish_markers()
            if self.publish_pose:
                self.pose_pub.publish(new_pose)
            self.prev_time = now

        self.pre_pose = new_pose

        # process command keys
        if not (status.R3 and status.R2 and status.L2):
            if history.new(status, "select"):
                self.command_pub.publish("SHARE_BBOX")
            if history.new(status, "circle"):
                self.set_color(marker, highlight=False)
                self.mode = self.MODE_MENU
                self.publish_menu(self.selecting_index)
            if history.new(status, "triangle"):
                #TODO preview
                rospy.logdebug("preview")
            if history.new(status, "cross"):
                # confirm and back to menu
                rospy.loginfo("Need confirm before quit.")
                self.confirm_index = 0
                self.publish_unsaved_menu(self.confirm_index)
                self.mode = self.MODE_UNSAVED
                self.publish_help()

    def init_marker(self):
        self.pre_marker = copy.deepcopy(self.current_marker)
        marker = self.current_marker
        if marker == None:
            rospy.logdebug("Creating new marker.")
            # create a new marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.type = self.type
            if self.current_index == 0:
                marker.ns = 'handle'
            elif self.current_index == 1:
                marker.ns = 'surface'
            elif self.current_index == 2:
                marker.ns = 'axis'

            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.6
            marker.id = 0
            self.set_color(marker, highlight=True)
            self.current_marker = marker
            self.markers.markers.append(self.current_marker)
        self.pre_pose.pose = marker.pose
        rospy.logdebug("Editing marker "+str(marker.ns))

    def set_marker_cb(self, box):
        marker = self.current_marker
        if marker == None:
            ros.logwarn("Trying to set marker of null value")
            return
        pose = PoseStamped()
        pose.header.frame_id = box.header.frame_id
        pose.pose = box.pose
        self.pre_pose = self.tf_listener.transformPose(self.frame_id, pose)

        marker.pose = self.pre_pose.pose
        marker.scale = box.dimensions
        if self.lock_xy:
            if marker.scale.y < marker.scale.x:
                marker.pose.position.x -= 0.5 * (marker.scale.x - marker.scale.y)
                marker.scale.y = marker.scale.x
            else:
                marker.pose.position.y += 0.5 * (marker.scale.x - marker.scale.y)
                marker.scale.x = marker.scale.y
            self.pre_pose.pose = marker.pose
        self.markers_pub.publish(self.markers)
        if self.publish_pose:
            pose.header.stamp = rospy.Time(0)
            self.pose_pub.publish(self.pre_pose)

    def publish_markers(self):
        t = rospy.Time(0)
        for m in self.markers.markers:
            m.header.stamp = t
        self.marker_array_pub.publish(self.markers)

    def publish_help(self):
        text = OverlayText()
        text.width = 400
        #text.height = 600
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        if self.mode == self.MODE_MENU:
            text.height = 170
            text.text = """Joy Control Help
Up/Down: Choose item
Triangle: Show markers' route
Triangle + Up/Down: Change order
Triangle + R1: [WIP] Preview IK for current marker
Square: Delete current marker
Circle: Add new / Edite existing marker
Start: [WIP] Execute 
            """
        elif self.mode == self.MODE_MARKER:
            text.height = 342
            text.text = """Joy Control Help
Left Analog: Translate xy
D-pad: Rotate pitch/roll
R1 + Left/Right: Rotate yaw
L1 + Up/Down: Change marker hight (z)
L1 + Left/Right: Cchange marker y size
L2/R2: Translate z
Square(Hold): Move faster

Right Analog: yaw/pitch of camera position
R3(Hold): suppressing buttons/sticks for controlling pose
R3 + L2+R2: enable follow view mode

Select: [WIP] Load from bounding box
Triangle: [WIP] Preview IK
Circle: Save and goto the next
Cross: Quit (without saving)
            """
        elif self.mode == self.MODE_UNSAVED or self.mode == self.MODE_DELETE:
            text.height = 77
            text.text = """Joy Control Help
Up/Down: Choose
Circle: Confirm
Cross: Quit (without saving)
            """
        self.help_pub.publish(text)

    def enable(self):
        rospy.logdebug("Enabled")
        #self.start()
        # TODO save the previous pose?
        #self.selecting_index = 0
        #self.switch_marker(0)
        #self.publish_menu(self.selecting_index, close=False)

    def disable(self):
        rospy.logdebug("Disabled")
        #self.publish_menu(self.selecting_index, close=True)
        # TODO Back to main menu

def main():
    global ma_manager
    rospy.sleep(1)
    rospy.init_node('joy_marker_array')
    ma_manager = JoyMarkerArray()
    result = g_manager.start()
    if not result:
        rospy.logfatal("Fatal Error")
        return False
    else:
        rospy.spin()

if __name__ == '__main__':
    main()
