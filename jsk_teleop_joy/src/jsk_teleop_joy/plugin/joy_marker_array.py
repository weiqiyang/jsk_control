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

from jsk_rviz_plugins.msg import OverlayMenu, OverlayText
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from joy_rviz_view_controller import RVizViewController

def signedSquare(val):
 if val > 0:
   sign = 1
 else:
   sign = -1
 return val * val * sign

class JoyMarkerArray(JSKJoyPlugin):
  '''
Usage:
Check publishHelp() for controller configurations.

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
  MODE_MARKER = 1
  MODE_DELETE = 2
  MODE_UNSAVED = 3
  mode = 0

  markers = MarkerArray()
  menu_list = ['Add new ...']
  isClosed = True
  next_id = 0


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
    self.title = self.getArg('title', 'MarkerList')
    self.pose_pub = rospy.Publisher(self.getArg('marker_pose', 'marker_pose'),
                                      PoseStamped, queue_size=10)
    self.menu_pub = rospy.Publisher(self.getArg('menu', 'dynamic_menu'),
                                      OverlayMenu, queue_size=10)
    self.marker_array_pub = rospy.Publisher(self.getArg('marker_array', 'marker_array'),
                                      MarkerArray, queue_size=10)
    self.help_pub = rospy.Publisher(self.getArg('help_text', 'help_text'),
                                      OverlayText, queue_size=1)
    self.frame_id = self.getArg('frame_id', 'map')
    self.lock_xy = self.getArg('lock_xy', False)
    self.type = self.getArg('type', 3)
    self.namespace = self.getArg('namespace', 'joy_markers')
    self.show_label = self.getArg('show_label', False)

    #TODO self.loadMarkers()
    self.start()

  def start(self):
    self.publishMenu(0, close=True) # close menu anyway
    if len(self.markers.markers) == 0:
      rospy.loginfo('The marker array is now empty')
      self.current_marker = None
    else:
      self.current_marker = self.markers.markers[0]
    return True

  def publishMenu(self, index, close=False):
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
          if self.moveItemUp(self.markers.markers, self.selecting_index):
            self.moveItemUp(self.menu_list, self.selecting_index)
            self.selecting_index = self.selecting_index  - 1
            self.publishMenu(self.selecting_index)
            self.switchMarker(self.selecting_index)
            self.showRoute()
        elif history.new(status, "down") or history.new(status, "left_analog_down"):
          if self.moveItemDown(self.markers.markers, self.selecting_index):
            self.moveItemDown(self.menu_list, self.selecting_index)
            self.selecting_index = self.selecting_index  + 1
            self.publishMenu(self.selecting_index)
            self.switchMarker(self.selecting_index)
            self.showRoute()
        elif history.new(status, "R1"):
          #TODO preview ik
          rospy.logdebug("preview ik")
        if not latest.triangle:
          #TODO show marker route (lifetime 5s)
          rospy.logdebug("show route")
          self.showRoute()
      else:
        if history.new(status, "down") or history.new(status, "left_analog_down"):
          self.selecting_index = self.selecting_index + 1
          if self.selecting_index >= len(self.menu_list):
            self.selecting_index = 0
          self.publishMenu(self.selecting_index)
          self.switchMarker(self.selecting_index)
        elif history.new(status, "up") or history.new(status, "left_analog_up"):
          self.selecting_index = self.selecting_index - 1
          if self.selecting_index < 0:
            self.selecting_index = len(self.menu_list) - 1
          self.publishMenu(self.selecting_index)
          self.switchMarker(self.selecting_index)
        elif history.new(status, "circle"):
          # close menu and edit
          self.publishMenu(self.selecting_index, close=True)
          self.initMarker()
          self.mode = self.MODE_MARKER
        elif history.new(status, "square"):
          # delete current marker
          if not self.current_marker == None:
            self.publishMenu(self.selecting_index, close=True)
            self.confirm_index = 0
            self.publishDeleteMenu(self.confirm_index)
            self.mode = self.MODE_DELETE
        elif history.new(status, "start"):
          #TODO execute?
          rospy.logdebug("start pressed")
        else:
          self.publishMenu(self.selecting_index)
        self.publishHelp()
    elif self.mode == self.MODE_MARKER:
      self.markerJoyCB(status, history)
    elif self.mode == self.MODE_DELETE:
      if history.new(status, "down") or history.new(status, "left_analog_down") \
        or history.new(status, "up") or history.new(status, "left_analog_up"):
        self.confirm_index = 1 - self.confirm_index
        self.publishDeleteMenu(self.confirm_index)
      elif history.new(status, "circle") and self.confirm_index == 1:
        self.publishDeleteMenu(self.confirm_index, close=True)
        self.deleteMarker(self.current_index)
        self.mode = self.MODE_MENU
        self.publishHelp()
        self.switchMarker(self.selecting_index)
        self.publishMarkers()
      elif history.new(status, "cross") \
        or (history.new(status, "circle") and self.confirm_index == 0):
        self.publishUnsavedMenu(self.confirm_index, close=True)
        self.mode = self.MODE_MENU
        self.publishHelp()
    elif self.mode == self.MODE_UNSAVED:
      if history.new(status, "down") or history.new(status, "left_analog_down") \
        or history.new(status, "up") or history.new(status, "left_analog_up"):
        self.confirm_index = 1 - self.confirm_index
        self.publishUnsavedMenu(self.confirm_index)
      elif history.new(status, "circle") and self.confirm_index == 1:
        self.publishUnsavedMenu(self.confirm_index, close=True)
        if self.pre_marker == None:
          self.deleteMarker(self.current_index)
          self.current_marker = None
          self.current_index -= 1
        else:
          # restore pre_marker
          self.current_marker = self.pre_marker
          self.markers.markers[self.current_index] = self.current_marker
          self.setColor(self.current_marker, highlight=False)
        self.mode = self.MODE_MENU
        self.publishHelp()
        self.switchMarker(self.selecting_index)
        self.publishMarkers()
      elif history.new(status, "cross") \
        or (history.new(status, "circle") and self.confirm_index == 0):
        self.publishUnsavedMenu(self.confirm_index, close=True)
        self.mode = self.MODE_MARKER
        self.publishHelp()

  def switchMarker(self, index):
    self.current_index = index
    if self.current_index > len(self.markers.markers):
      self.current_index = 0
    elif self.current_index < 0:
      self.current_index = len(self.markers.markers)

    if not self.current_marker == None:
      self.setColor(self.current_marker, highlight=False)
      #rospy.logdebug("Set marker " + str(self.current_marker.id) + " highlight off.")
    if self.current_index == len(self.markers.markers):
      #rospy.logdebug("Marker " + str(self.current_index) + " is None.")
      self.current_marker = None
    else:
      self.current_marker = self.markers.markers[self.current_index]
      #rospy.logdebug("Set marker " + str(self.current_marker.id) + " highlight on.")
      self.setColor(self.current_marker, highlight=True)
    self.publishMarkers()

  def deleteMarker(self, index):
    rospy.logdebug("delete marker " + str(self.current_index))
    self.markers.markers[index].action = Marker.DELETE
    self.publishMarkers()
    self.markers.markers.remove(self.markers.markers[index])
    self.menu_list.remove(self.menu_list[index])

  def moveItemUp(self, li, index):
    rospy.logdebug("move up index " + str(index))
    if index > 0 and index < len(li):
      temp = li[index]
      li[index] = li[index-1]
      li[index-1] = temp
      return True
    else:
      return False

  def moveItemDown(self, li, index):
    rospy.logdebug("move down index " + str(index))
    if index >= 0 and index < len(li)-1:
      temp = li[index]
      li[index] = li[index+1]
      li[index+1] = temp
      return True
    else:
      return False

  def showRoute(self):
    route = Marker()
    route.header.frame_id = self.frame_id
    route.lifetime.secs = 5
    # LINE_STRIP=4
    route.type = 4
    route.ns = self.namespace + "_route"
    route.scale.x = 0.1
    for m in self.markers.markers:
      route.points.append(m.pose.position)
    self.setColor(route, highlight=True)
    self.markers.markers.append(route)
    self.publishMarkers()
    self.markers.markers.pop()

  def setColor(self, marker, highlight=False):
    if highlight:
      marker.color.r = 0.0
      marker.color.g = 1.0
    else:
      marker.color.r = 0.6
      marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

  def publishDeleteMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = ["No", "Yes"]
    menu.current_index = index
    menu.title = "Delete current marker? (Cannot undo)"
    self.isClosed = False
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
      self.isClosed = True
    self.menu_pub.publish(menu)

  def publishUnsavedMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = ["No", "Yes"]
    menu.current_index = index
    menu.title = "Quit edit mode? (Unsaved markers will be aborted)"
    self.isClosed = False
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
      self.isClosed = True
    self.menu_pub.publish(menu)

  def markerJoyCB(self, status, history):
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
      x_diff = signedSquare(status.left_analog_y) / scale
      y_diff = signedSquare(status.left_analog_x) / scale
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
    xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                         local_move)
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
        # keep minimun scale value
        if not marker.scale.x > 0:
          marker.scale.x = DSCALE
        if not marker.scale.y > 0:
          marker.scale.y = DSCALE
        if not marker.scale.z > 0:
          marker.scale.z = DSCALE
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
      self.publishMarkers()
      if self.publish_pose:
        self.pose_pub.publish(new_pose)
      self.prev_time = now

    self.pre_pose = new_pose

    # process command keys
    if not (status.R3 and status.R2 and status.L2):
      if status.circle and not latest.circle:
        self.setColor(marker, highlight=False)
        if marker.id == self.next_id:
          # add current marker and goto the next
          self.next_id += 1
        self.current_index += 1
        self.switchMarker(self.current_index)
        self.initMarker()
      if status.triangle and not latest.triangle:
        #TODO preview
        rospy.logdebug("preview")
      if status.cross and not latest.cross:
        # confirm and back to menu
        rospy.loginfo("Need confirm before quit.")
        self.confirm_index = 0
        self.publishUnsavedMenu(self.confirm_index)
        self.mode = self.MODE_UNSAVED
        self.publishHelp()

  def initMarker(self):
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
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
      else:
        # when the array is not empty, start from the last marker
        marker = copy.deepcopy(self.markers.markers[self.current_index-1])
        rospy.logdebug("Copying marker " + str(self.current_index-1))
      marker.id = self.next_id
      self.setColor(marker, highlight=True)
      self.current_marker = marker
      self.menu_list.insert(self.current_index, "Marker"+str(marker.id))
      self.markers.markers.append(self.current_marker)
    self.pre_pose.pose = marker.pose
    rospy.logdebug("Editing marker "+str(marker.id))

  def publishMarkers(self):
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
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
      self.marker_array_pub.publish(labels)
    self.marker_array_pub.publish(self.markers)

  def publishHelp(self):
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
    #self.switchMarker(0)
    #self.publishMenu(self.selecting_index, close=False)

  def disable(self):
    rospy.logdebug("Disabled")
    #self.publishMenu(self.selecting_index, close=True)
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
