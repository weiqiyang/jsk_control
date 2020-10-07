#!/usr/bin/env python

import math
import numpy
import os
import sys
import time
import copy

import rospy
import rosbag
import tf

from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA, Float32
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_rviz_plugins.msg import OverlayMenu, OverlayText
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from confirm_menu import YesNoMenu
from joy_rviz_view_controller import RVizViewController

def signed_square(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign

class JoyMarkerArray(RVizViewController):
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
bbox_topic[String, default: multi_euclidean_cluster_point_indices_decomposer/boxes]: bbox topic name to subscribe for initializing marker array
    '''
    STATE_INITIALIZATION = 1
    STATE_RUNNING = 2
    STATE_WAIT_FOR_JOY = 3

    MODE_MENU = 0
    MODE_MARKER = 1
    MODE_DELETE = 2
    MODE_UNSAVED = 3
    MODE_LOAD = 4
    MODE_MANIP = 5
    mode = 0

    assoc_flag = False

    markers = MarkerArray()
    pending_markers = []
    menu_list = ['Add new ...']
    isClosed = True
    next_id = 0


    def __init__(self, name, args):
        RVizViewController.__init__(self, name, args)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.prev_time = rospy.Time.from_sec(time.time())
        self.publish_pose = self.getArg('publish_pose', True)
        self.supportFollowView(True)
        self.current_index = 0
        self.selecting_index = 0
        self.title = self.getArg('title', 'MarkerList')
        menu_topic = self.getArg('menu', 'dynamic_menu')

        self.pose_pub = rospy.Publisher(self.getArg('marker_pose', 'marker_pose'),
                                        PoseStamped, queue_size=1)
        self.target_pub = rospy.Publisher(self.getArg('target_pose', 'target_pose'),
                                        PoseStamped, queue_size=1)
        self.menu_pub = rospy.Publisher(menu_topic,
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
        self.bbox_topic = self.getArg('bbox_topic',
                                    "multi_euclidean_cluster_point_indices_decomposer/boxes")
        self.tf_listener = tf.TransformListener()
        self.manip_pose = None

        delete_cap = "Delete current marker? (This cannot be undone)"
        self.delete_menu = YesNoMenu(menu_topic, delete_cap, self.delete_yes, self.delete_no)
        unsaved_cap = "Quit edit mode? (Unsaved markers will be aborted)"
        self.unsaved_menu = YesNoMenu(menu_topic, unsaved_cap, self.unsaved_yes, self.unsaved_no)
        load_cap = "Do you want to save the loaded markers?"
        self.load_menu = YesNoMenu(menu_topic, load_cap, self.load_yes, self.load_no)

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
                if history.new(status, "up") or history.new(status, "left_analog_up"):
                    if self.move_item_up(self.markers.markers, self.selecting_index):
                        self.move_item_up(self.menu_list, self.selecting_index)
                        self.selecting_index = self.selecting_index - 1
                        self.publish_menu(self.selecting_index)
                        self.switch_marker(self.selecting_index)
                        self.show_route()
                elif history.new(status, "down") or history.new(status, "left_analog_down"):
                    if self.move_item_down(self.markers.markers, self.selecting_index):
                        self.move_item_down(self.menu_list, self.selecting_index)
                        self.selecting_index = self.selecting_index + 1
                        self.publish_menu(self.selecting_index)
                        self.switch_marker(self.selecting_index)
                        self.show_route()
                elif history.new(status, "R1"):
                    #TODO preview ik
                    rospy.logdebug("preview ik")
                if not latest.triangle:
                    # Show marker route (lifetime 5s)
                    rospy.logdebug("show route")
                    self.show_route()
            elif status.L1:
                if history.new(status, "select"):
                    bag = rosbag.Bag('marker.bag')
                    last_pt = []
                    for topic, msg, t in bag.read_messages(topics=['markers', 'pose']):
                        last_pt.append(msg)
                    bag.close()
                    l = len(last_pt)
                    rospy.loginfo("msg length: "+str(l))
                    if l > 1:
                        self.markers = last_pt[l-2]
                        rospy.loginfo("Load "+str(len(self.markers.markers)) + " markers.")
                        self.manip_pose = last_pt[l-1]
                        #reset marker list
                        self.next_id = 0
                        self.current_index = 0
                        self.current_marker = None
                        self.menu_list = ['Add new ...']
                        for marker in self.markers.markers:
                            marker.id = self.next_id
                            marker.ns = self.namespace
                            self.set_color(marker, highlight=False)
                            self.menu_list.insert(self.current_index, "Marker"+str(marker.id))
                            self.next_id += 1
                            self.current_index += 1
                            self.current_marker = marker
                        self.current_index = 0
                        self.publish_markers()
                        #rospy.loginfo("Menu length"+str(len(self.menu_list)))
                elif history.new(status, "start"):
                    bag = rosbag.Bag('marker.bag', 'w')
                    try:
                        bag.write('markers', self.markers)
                        bag.write('pose', self.manip_pose)
                        rospy.loginfo("saved")
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
                    self.init_marker()
                    self.mode = self.MODE_MARKER
                elif history.new(status, "square"):
                    # delete current marker
                    if not self.current_marker == None:
                        self.publish_menu(self.selecting_index, close=True)
                        self.delete_menu.publish_menu(1)
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
            self.delete_menu.joy_cb(status, history)
        elif self.mode == self.MODE_UNSAVED:
            self.unsaved_menu.joy_cb(status, history)
        elif self.mode == self.MODE_LOAD:
            self.load_menu.joy_cb(status, history)
        elif self.mode == self.MODE_MANIP:
            self.manip_joy_cb(status, history)

    def unsaved_yes(self):
        if self.pre_marker == None:
            self.delete_marker(self.current_index)
            self.current_marker = None
            self.current_index -= 1
        else:
            # restore pre_marker
            self.current_marker = self.pre_marker
            self.markers.markers[self.current_index] = self.current_marker
            self.set_color(self.current_marker, highlight=False)
        self.switch_marker(self.selecting_index)
        self.publish_markers()
        self.mode = self.MODE_MENU
        self.publish_help()

    def unsaved_no(self):
        self.mode = self.MODE_MARKER
        self.publish_help()

    def load_yes(self):
        self.mode = self.MODE_MENU
        self.publish_help()
        self.publish_menu(self.current_index)

    def load_no(self):
        # TODO delete loaded markers
        self.mode = self.MODE_MENU
        self.publish_help()
        self.publish_menu(self.current_index)

    def switch_marker(self, index):
        self.current_index = index
        if self.current_index > len(self.markers.markers):
            self.current_index = 0
        elif self.current_index < 0:
            self.current_index = len(self.markers.markers)

        if not self.current_marker == None:
            self.set_color(self.current_marker, highlight=False)
            #rospy.logdebug("Set marker " + str(self.current_marker.id) + " highlight off.")
        if self.current_index == len(self.markers.markers):
            #rospy.logdebug("Marker " + str(self.current_index) + " is None.")
            self.current_marker = None
        else:
            self.current_marker = self.markers.markers[self.current_index]
            #rospy.logdebug("Set marker " + str(self.current_marker.id) + " highlight on.")
            self.set_color(self.current_marker, highlight=True)
        self.publish_markers()

    def delete_marker(self, index):
        rospy.loginfo("delete marker " + str(self.current_index))
        self.markers.markers[index].action = Marker.DELETE
        self.publish_markers()
        self.markers.markers.remove(self.markers.markers[index])
        self.menu_list.remove(self.menu_list[index])

    def move_item_up(self, li, index):
        rospy.logdebug("move up index " + str(index))
        if index > 0 and index < len(li):
            temp = li[index]
            li[index] = li[index-1]
            li[index-1] = temp
            return True
        else:
            return False

    def move_item_down(self, li, index):
        rospy.logdebug("move down index " + str(index))
        if index >= 0 and index < len(li)-1:
            temp = li[index]
            li[index] = li[index+1]
            li[index+1] = temp
            return True
        else:
            return False

    def show_route(self):
        route = Marker()
        route.header.frame_id = self.frame_id
        route.lifetime.secs = 5
        # LINE_STRIP=4
        route.type = 4
        route.ns = self.namespace + "_route"
        route.scale.x = 0.01
        for m in self.markers.markers:
            route.points.append(m.pose.position)
        self.set_color(route, highlight=True)
        self.markers.markers.append(route)
        self.publish_markers()
        self.markers.markers.pop()

    def set_color(self, marker, highlight=False):
        if highlight:
            marker.color.r = 0.0
            marker.color.g = 1.0
        else:
            marker.color.r = 0.6
            marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    def delete_yes(self):
        self.delete_marker(self.current_index)
        self.switch_marker(self.selecting_index)
        self.publish_markers()
        self.mode = self.MODE_MENU
        self.publish_help()

    def delete_no(self):
        self.mode = self.MODE_MENU
        self.publish_help()

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
        marker = self.current_marker
        self.pose_joy_cb(status, history)
        marker.pose = self.pre_pose.pose
        # scale
        DSCALE = 0.002
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
                self.pose_pub.publish(self.pre_pose)
            self.prev_time = now

        # process command keys
        if not (status.R3 and status.R2 and status.L2 and status.L1):
            if history.new(status, "select"):
                #command_pub.publish("SHARE_BBOX")
                try:
                    self.set_color(marker, highlight=False)
                    rospy.loginfo("Subscribing bounding boxes...")
                    boxes = rospy.wait_for_message(self.bbox_topic,
                                                    BoundingBoxArray, timeout=10)
                    rospy.loginfo("Adding " + str(len(boxes.boxes)) + " boxes to marker array...")
                    local_id = 0
                    if marker.id == self.next_id:
                        # abort current marker
                        self.delete_marker(self.current_index)
                        self.current_index -= 1
                    for box in boxes.boxes:
                        self.current_index = len(self.markers.markers)
                        self.current_marker = None
                        self.init_marker()
                        marker = self.current_marker
                        marker.id = self.next_id# + local_id
                        #local_id += 1
                        self.next_id += 1
                        self.set_marker_by_box(marker, box)
                        self.pending_markers.append(self.current_marker)
                    # self.next_id += local_id
                    self.publish_markers()
                    #TODO Confirm adding
                    self.load_menu.publish_menu(0)
                    self.mode = self.MODE_LOAD
                except rospy.exceptions.ROSException:
                    rospy.logwarn("Time out. Failed to get bounding boxes from " + self.bbox_topic)
            if history.new(status, "circle"):
                self.set_color(marker, highlight=False)
                if self.pre_marker == None:
                    self.next_id += 1
                self.current_index += 1
                self.switch_marker(self.current_index)
                self.init_marker()
            if history.new(status, "triangle"):
                #TODO preview
                if self.current_marker.id == self.next_id:
                    rospy.logwarn("Please save marker before manipulation.")
                else:
                    self.init_manip_pose(self.current_marker)
                    self.mode = self.MODE_MANIP
                    self.publish_help()
            if history.new(status, "cross"):
                # confirm and back to menu
                rospy.loginfo("Need confirm before quit.")
                self.unsaved_menu.publish_menu(1)
                self.mode = self.MODE_UNSAVED
                self.publish_help()

    def init_marker(self):
        self.pre_marker = copy.deepcopy(self.current_marker)
        marker = self.current_marker
        if marker == None:
            rospy.logdebug("Creating new marker.")
            # create a new marker
            if len(self.markers.markers) == 0:
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.type = self.type
                marker.ns = self.namespace

                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.1
            else:
                # when the array is not empty, start from the last marker
                marker = copy.deepcopy(self.markers.markers[self.current_index-1])
                rospy.logdebug("Copying marker " + str(self.current_index-1))
            marker.id = self.next_id
            self.set_color(marker, highlight=True)
            self.current_marker = marker
            self.menu_list.insert(self.current_index, "Marker"+str(marker.id))
            self.markers.markers.append(self.current_marker)
        self.pre_pose.pose = marker.pose
        rospy.logdebug("Editing marker "+str(marker.id))

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

    def set_marker_cb(self, box):
        marker = self.current_marker
        if marker == None:
            ros.logwarn("Trying to set marker of null value")
            return
        self.set_marker_by_box(marker, box)
        self.markers_pub.publish(self.markers)
        if self.publish_pose:
            pose.header.stamp = rospy.Time(0)
            self.pose_pub.publish(self.pre_pose)

    def set_marker_by_box(self, marker, box):
        pose = PoseStamped()
        pose.header.frame_id = box.header.frame_id
        pose.pose = box.pose
        self.pre_pose = self.tf_listener.transformPose(self.frame_id, pose)

        marker.pose = self.pre_pose.pose
        marker.scale = box.dimensions
        if self.lock_xy:
            local_diff = 0.5 * (marker.scale.x - marker.scale.y)

            if marker.scale.y < marker.scale.x:
                diff_q = tf.transformations.quaternion_from_euler(math.pi/2, 0.0, 0.0)
                marker.scale.y = marker.scale.x
            else:
                diff_q = tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0)
                marker.scale.x = marker.scale.y
            marker.pose = self.translate_pose(marker.pose, local_diff, diff_q)
            self.pre_pose.pose = marker.pose

    def translate_pose(self, pre_pose, dist, diff_q):
        pose = copy.deepcopy(pre_pose)
        q = numpy.array((pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w))

        local_xyz = numpy.array((0.0, 0.0, dist, 1.0))
        new_q = tf.transformations.quaternion_multiply(q, diff_q)

        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(new_q),
                                                 local_xyz)
        pose.position.x += xyz_move[0]
        pose.position.y += xyz_move[1]
        pose.position.z += xyz_move[2]
        return pose

    def publish_markers(self):
        t = rospy.Time(0)
        for m in self.markers.markers:
            m.header.stamp = t
        if self.show_label:
            labels = copy.deepcopy(self.markers)
            for m, txt in zip(labels.markers, self.menu_list):
                # no label for route line
                if m.type == 4:
                    break
                # TEXT_VIEW_FACING=9
                m.type = 9
                m.ns = self.namespace + "_label"
                m.text = txt
                m.pose.position.z += m.scale.z
                m.scale.x = 0.1
                m.scale.y = 0.1
                m.scale.z = 0.1
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 1.0
                m.color.a = 1.0
            self.marker_array_pub.publish(labels)
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
Triangle: Switch to manip mode
Circle: Save and goto the next
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
