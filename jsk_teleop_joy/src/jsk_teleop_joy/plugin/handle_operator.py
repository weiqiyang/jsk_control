#!/usr/bin/env python

import math
import numpy
import copy

import rospy
import rosbag
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

class HandleOperator(RVizViewController):
    '''
Usage:
Check publish_help() for controller configurations.

Args:
publish_pose [Boolean, default: True]: publish pose or not
title [String, default: Handle Operator]: title of the menu
marker_pose [String, default: marker_pose]: topic name to publishing marker pose
menu [String, default: dynamic_menu]: topic name to publish the menu
help_text [String, default: help_text]: topic name to publish overlay help text
lock_xy [Boolean, default: False]: to keep x,y scale equal or not
frame_id [String, default: map]: frame_id of publishing pose
namespace [String, default: joy_markers]: namespace to publish the marker
type [Int, default: 3]: type of the marker, cylinder by default
show_label [Boolean, default: False]: display labels for marks or not
sample_rad [float, defaulf: 0.3]: difference in rad of samples
    '''
    STATE_INITIALIZATION = 1
    STATE_RUNNING = 2
    STATE_WAIT_FOR_JOY = 3

    MODE_MENU = 0
    MODE_HANDLE = 1
    MODE_MARKER = 2
    MODE_AXIS = 3
    MODE_OPERATE = 4
    MODE_MANIP = 5
    MODE_UNSAVED = 10
    MODE_DELETE = 11

    # deleted 'Set Door Surface'
    menu_list = ['Set Handle', 'Set Door Axis', 'Operate']

    def __init__(self, name, args):
        RVizViewController.__init__(self, name, args)
        self.supportFollowView(True)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.prev_time = rospy.Time.from_sec(time.time())
        self.publish_pose = self.getArg('publish_pose', True)
        self.current_index = 0
        self.confirm_index = 0
        self.selecting_index = 0
        self.markers = MarkerArray()
        self.title = self.getArg('title', 'Handle Operator')
        self.pose_pub = rospy.Publisher(self.getArg('marker_pose', 'marker_pose'),
                                        PoseStamped, queue_size=10)
        self.target_pub = rospy.Publisher(self.getArg('target_pose', 'target_pose'),
                                        PoseStamped, queue_size=1)
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
        self.sample_rad = self.getArg('sample_rad', 0.3)

        self.mode = 0
        self.angle = 0.0
        self.handle_marker = None
        self.axis_marker = None
        self.manip_pose = None

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
            elif status.L1 and history.new(status, "start"):
                bag = rosbag.Bag('marker.bag', 'w')
                try:
                    manip_pose = PoseStamped()
                    if self.handle_marker == None:
                        manip_pose = None
                    else:
                        manip_pose.header = self.handle_marker.header
                        manip_pose.pose = self.handle_marker.pose
                    marker_array = MarkerArray()
                    for marker in self.markers.markers:
                        if marker.ns == "handle":
                            marker_array.markers.insert(0, marker)
                        elif marker.ns == "sample":
                            marker_array.markers.append(marker)
                    bag.write('markers', marker_array)
                    rospy.loginfo("Saved " + str(len(marker_array.markers)) + " markers.")
                    bag.write('pose', manip_pose)
                finally:
                    bag.close()
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
                    if self.selecting_index == 2:
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
            self.operate_joy_cb(status, history)
        elif self.mode == self.MODE_MANIP:
            self.manip_joy_cb(status, history)

    def switch_marker(self, index):
        self.current_index = index
        ns = ''
        rospy.loginfo("current index: " + str(index))
        if index == 0:
            ns = 'handle'
        elif index == 1:
            ns = 'axis'
        elif index == 2:
            ns = 'target'
        rospy.loginfo("current ns: " + ns)

        if not self.current_marker == None:
            self.set_color(self.current_marker, highlight=False)

        self.current_marker = None
        for m in self.markers.markers:
            if m.ns == ns:
                self.current_marker = m
                self.set_color(self.current_marker, highlight=True)

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

    def pose_joy_cb(self, status, history):
        pre_pose = self.pre_pose
        if history.length() > 0:
            latest = history.latest()
            if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
                self.followView(not self.followView())
        if self.control_view:
            RVizViewController.joyCB(self, status, history)
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
            local_move = numpy.array((x_diff, y_diff, z_diff * z_scale, 1.0))
        else:
            local_move = numpy.array((0.0, 0.0, 0.0, 1.0))

        q = numpy.array((pre_pose.pose.orientation.x,
                         pre_pose.pose.orientation.y,
                         pre_pose.pose.orientation.z,
                         pre_pose.pose.orientation.w))
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                             local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]

        # rotate
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.02
        if not status.R3 and not status.L1:
            if status.R1:
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

        self.pre_pose = new_pose

    def marker_joy_cb(self, status, history):
        pre_pose = self.pre_pose
        marker = self.current_marker
        self.pose_joy_cb(status, history)
        new_pose = self.pre_pose
        marker.pose = self.pre_pose.pose

        # scale
        if not status.R3 and status.L1:
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

    def operate_joy_cb(self, status, history):
        if self.handle_marker == None:
            rospy.logwarn("Handle not defined")
            self.mode = self.MODE_MENU
            self.publish_menu(self.current_index)
        pre_pose = self.pre_pose
        marker = self.current_marker
        if self.current_marker == None:
            marker = copy.deepcopy(self.handle_marker)
            marker.ns = "target"
            self.set_color(marker, highlight=True)
            self.markers.markers.append(marker)
            self.current_marker = marker

        marker.scale = self.handle_marker.scale
        marker.pose.orientation = self.handle_marker.pose.orientation
        axis = self.axis_marker.pose.orientation
        drct = numpy.array((2 * (axis.x * axis.z - axis.y * axis.w),
                            2 * (axis.y * axis.z - axis.x * axis.w),
                            1 - 2 * (axis.x * axis.x + axis.y * axis.y)))
        origin = self.axis_marker.pose.position
        p0 = numpy.array((origin.x, origin.y, origin.z))
        handle_p = numpy.array((self.handle_marker.pose.position.x,
                                self.handle_marker.pose.position.y,
                                self.handle_marker.pose.position.z, 1.0))

        DELTA = 0.02
        if history.length() > 0:
            latest = history.latest()
            if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
                self.followView(not self.followView())
        if self.control_view:
            RVizViewController.joyCB(self, status, history)

        if not status.R3:
            if status.up:
                if status.square:
                    self.angle = self.angle + DELTA * 5
                elif history.all(lambda s: s.up):
                    self.angle = self.angle + DELTA * 2
                else:
                    self.angle = self.angle + DELTA
            elif status.down:
                if status.square:
                    self.angle = self.angle - DELTA * 5
                elif history.all(lambda s: s.down):
                    self.angle = self.angle - DELTA * 2
                else:
                    self.angle = self.angle - DELTA
            if self.angle > numpy.pi:
                self.angle = self.angle - numpy.pi * 2
            elif self.angle < - numpy.pi:
                self.angle = self.angle + numpy.pi * 2
            rot_m = tf.transformations.rotation_matrix(self.angle, drct, p0)
            tp = numpy.dot(rot_m, handle_p)
            marker.pose.position.x = tp[0]
            marker.pose.position.y = tp[1]
            marker.pose.position.z = tp[2]
            if history.new(status, "cross"):
                self.mode = self.MODE_MENU
                self.publish_menu(self.current_index)
            elif history.new(status, "circle"):
                self.generate_samples()
            if history.new(status, "triangle"):
                self.generate_samples()
                self.init_manip_pose(self.current_marker)
                self.mode = self.MODE_MANIP
                self.publish_help()

        self.publish_markers()

    def generate_samples(self):
        for m in self.markers.markers:
            if m.ns == "sample":
                m.action = Marker.DELETE
        self.publish_markers()
        d_list = []
        for m in self.markers.markers:
            if m.ns == "sample":
                d_list.append(m)
        for m in d_list:
            self.markers.markers.remove(m)

        handle_p = numpy.array((self.handle_marker.pose.position.x,
                                self.handle_marker.pose.position.y,
                                self.handle_marker.pose.position.z, 1.0))
        axis = self.axis_marker.pose.orientation
        drct = numpy.array((2 * (axis.x * axis.z - axis.y * axis.w),
                            2 * (axis.y * axis.z - axis.x * axis.w),
                            1 - 2 * (axis.x * axis.x + axis.y * axis.y)))
        origin = self.axis_marker.pose.position
        p0 = numpy.array((origin.x, origin.y, origin.z))

        a_sign = 1
        if self.angle < 0.0:
            a_sign = -1
        sample_id = 0
        current_rad = self.sample_rad
        while current_rad < self.angle * a_sign:
            sample_marker = copy.deepcopy(self.handle_marker)
            sample_marker.ns = "sample"
            self.set_color(sample_marker, highlight=True)
            sample_marker.color.a = 0.7
            sample_marker.id = sample_id
            mat = tf.transformations.rotation_matrix(current_rad * a_sign, drct, p0)
            sp = numpy.dot(mat, handle_p)
            sample_marker.pose.position.x = sp[0]
            sample_marker.pose.position.y = sp[1]
            sample_marker.pose.position.z = sp[2]
            self.markers.markers.append(sample_marker)
            current_rad += self.sample_rad
            sample_id += 1
        sample_marker = copy.deepcopy(self.current_marker)
        sample_marker.ns = "sample"
        self.set_color(sample_marker, highlight=True)
        sample_marker.color.a = 0.7
        sample_marker.id = sample_id
        self.markers.markers.append(sample_marker)

    def init_manip_pose(self, marker):
        if self.manip_pose == None:
            self.manip_pose = PoseStamped()
            self.manip_pose.header = copy.deepcopy(marker.header)
            self.manip_pose.pose = copy.deepcopy(marker.pose)
        else:
            #TODO
            return

    def manip_joy_cb(self, status, history):
        self.pre_pose = self.manip_pose
        self.pose_joy_cb(status, history)
        self.manip_pose = copy.deepcopy(self.pre_pose)

        # process command keys
        if not (status.R3 and status.R2 and status.L2):
            if status.L1:
                # assoc mode
                if history.new(status, "up") or history.new(status, "left"):
                    self.selecting_index = self.selecting_index - 1
                    if self.selecting_index < 0:
                        self.selecting_index = len(self.markers.markers) - 1
                    self.switch_marker(self.selecting_index)
                elif history.new(status, "down") or history.new(status, "right"):
                    self.selecting_index = self.selecting_index + 1
                    if self.selecting_index >= len(self.markers.markers):
                        self.selecting_index = 0
                    self.switch_marker(self.selecting_index)
                if history.new(status, "triangle"):
                    if self.assoc_flag:
                        self.publish_marker_command(self.current_marker, "ASSOC_PREVIEW")
                    else:
                        rospy.logwarn("Please assoc current pose after initial preview.")
                if history.new(status, "square"):
                    # Reset manip pose
                    self.manip_pose = None
                    self.init_manip_pose(self.current_marker)
                if history.new(status, "start"):
                    self.publish_marker_command(self.current_marker, "ASSOC_EXECUTE")

            else:
                if history.new(status, "triangle"):
                    self.assoc_flag = False
                    self.publish_pose_command(self.manip_pose, "PREVIEW")
                elif history.new(status, "circle"):
                    self.assoc_flag = True
                    self.publish_marker_command(self.current_marker, "ASSOC")
                elif history.new(status, "start") and not status.select:
                    self.publish_marker_command(self.current_marker, "EXECUTE")
                # Delete reset manip for conflicting with ipega home setting (select+start)
                #elif history.new(status, "select"):
                #    self.publish_marker_command(self.current_marker, "RESET_MANIP")
                if history.new(status, "cross"):
                    self.mode = self.MODE_MENU
                    self.publish_menu(self.selecting_index)
                    self.publish_help()
                    self.switch_marker(self.selecting_index)
                    self.publish_markers()

        # publish at 30hz
        if self.publish_pose:
            now = rospy.Time.from_sec(time.time())
            # placement.time_from_start = now - self.prev_time
            if (now - self.prev_time).to_sec() > 1 / 30.0:
                self.pose_pub.publish(self.pre_pose)
                self.prev_time = now

    def publish_pose_command(self, pose, command):
        self.target_pub.publish(pose)
        self.command_pub.publish(command)

    def publish_marker_command(self, marker, command):
        pose = PoseStamped()
        pose.pose = marker.pose
        pose.header = marker.header
        self.publish_pose_command(pose, command)

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
                self.handle_marker = marker
            elif self.current_index == -1:
                marker.ns = 'surface'
            elif self.current_index == 1:
                marker.ns = 'axis'
                self.axis_marker = marker

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
Square: Delete current marker
Circle: Add new / Edite existing marker
Start: Execute Mode (Not yet)
            """
        elif self.mode == self.MODE_MARKER:
            text.height = 342
            text.text = """Joy Control Help
Left Analog: Translate xy
D-pad: Rotate pitch/roll
R1 + Left/Right: Rotate yaw
L1 + Up/Down: Change marker hight (z)
L1 + Left/Right: Change marker y size
L2/R2: Translate z
Square(Hold): Move faster

Right Analog: Yaw/pitch of camera position
R3(Hold) + arrow buttons/sticks: Move camera
R3 + L2+R2: Enable follow view mode

Select: Load from bounding box
Circle: Save and quit
Cross: Quit (without saving)
            """
        elif self.mode == self.MODE_OPERATE:
            text.height = 342
            text.text = """Joy Control Help
D-pad Up/Down: Move handle end postision
Square(Hold): Move faster

Right Analog: Yaw/pitch of camera position
R3(Hold) + arrow buttons/sticks: Move camera
R3 + L2+R2: Enable follow view mode

Select: Load from bounding box
Circle: Generate track
Triangle: Switch to manip mode
Cross: Quit (without saving)
            """
        elif self.mode == self.MODE_MANIP:
            text.height = 382
            text.text = """Joy Control Help
Left Analog: Translate xy
D-pad: Rotate pitch/roll
R1 + Left/Right: Rotate yaw
L1 + Up/Down: Change marker hight (z)
L1 + Left/Right: Change marker y size
L2/R2: Translate z
Square(Hold): Move faster

Right Analog: yaw/pitch of camera position
R3(Hold) + arrow buttons/sticks: Move camera
R3 + L2+R2: enable follow view mode

Triangle: Preview IK for current pose
Circle: Bind current pose to object
L1 + Triangle: Preview IK for current object
L1 + Circle: Excute current IK
Select: Reset robot pose
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
