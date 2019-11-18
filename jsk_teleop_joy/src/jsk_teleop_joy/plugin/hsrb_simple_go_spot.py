#!/usr/bin/env python

import math
import numpy

import rospy
import os
import sys

import imp
try:
  imp.find_module("sensor_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import tf.transformations

from jsk_hsr_startup import SpotMixin
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from jsk_rviz_plugins.msg import OverlayMenu
from status_history import StatusHistory

class HsrbSimpleGoSpot(JSKJoyPlugin):
  '''
Usage:
This plugin calls ROS services for HSR to get saved spots and send go_to_spot order.

Up/Down: choose from menu

circle/square/triangle: publish cooperating command

Args:
prefix [String, default: ]: prefix of the spots to be listed; it will be obmitted from the spots' name
title [String, default: prefix/SpotList]: title of the menu; it is the same as prefix by default; if prefix is an empty string, default value is 'SpotList'
topic [String, default: dynamic_menu]: topic name to publish the menu
command [String, default: command]: topic name for publishing the command
triangle_cmd [String, default: GS_TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd [String, default: GS_CIRCLE_CMD]: command text when triangle button is pressed
square_cmd [String, default: GS_SQUARE_CMD]: command text when triangle button is pressed
  '''
  STATE_INITIALIZATION = 1
  STATE_RUNNING = 2
  STATE_WAIT_FOR_JOY = 3

  MODE_PLUGIN = 0
  MODE_MENU = 1
  mode = 0

  item_instances = []

  def stateDiagnostic(self, stat):
    if self.state == self.STATE_INITIALIZATION:
      stat.summary(DiagnosticStatus.WARN,
                   "initializing JoyManager")
    elif self.state == self.STATE_RUNNING:
      stat.summary(DiagnosticStatus.OK, "running")
      stat.add("Joy stick type", str(self.JoyStatus))
    elif self.state == self.STATE_WAIT_FOR_JOY:
      stat.summary(DiagnosticStatus.WARN,
                   "waiting for joy message to detect joy stick type")
    return stat

  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.spot_mixin = SpotMixin()
    self.current_item_index = 0
    self.selecting_item_index = 0
    self.prefix = self.getArg('prefix','')
    self.title = self.getArg('title', self.prefix)
    if self.title == '':
      self.title = 'SpotList'
    self.menu_pub = rospy.Publisher(self.getArg('topic', 'dynamic_menu'),
                                      OverlayMenu, queue_size=10)
    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    self.triangle_cmd = self.getArg('triangle_cmd', 'GS_TRIANGLE_CMD')
    self.square_cmd = self.getArg('square_cmd', 'GS_SQUARE_CMD')
    self.circle_cmd = self.getArg('circle_cmd', 'GS_CIRCLE_CMD')

    self.loadItems()
    self.start()

  def loadItems(self):
    self.item_instances = []
    spots = self.spot_mixin.get_spots()
    start = len(self.prefix)
    for s in spots:
      if s.startswith(self.prefix):
        self.item_instances.append(s[start:])

  def switchItem(self, index):
    self.current_item_index = index
    if len(self.item_instances) <= self.current_item_index:
      self.current_item_index = 0
    elif self.current_item_index < 0:
      self.current_item_index = len(self.item_instances)
    self.current_item = self.item_instances[self.current_item_index]
    self.display(self.current_item)

  def start(self):
    self.publishMenu(0, close=True) # close menu anyway
    if len(self.item_instances) == 0:
      rospy.logfatal('no valid spots are loaded')
      return False
    self.current_item = self.item_instances[0]
    return True

  def publishMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = self.item_instances
    menu.current_index = index
    menu.title = self.title
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
    self.menu_pub.publish(menu)

  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
    if len(self.item_instances) > 0:
      if history.new(status, "down") or history.new(status, "left_analog_down"):
        self.selecting_item_index = self.selecting_item_index + 1
        if self.selecting_item_index >= len(self.item_instances):
          self.selecting_item_index = 0
        self.publishMenu(self.selecting_item_index)
        self.switchItem(self.selecting_item_index)
      elif history.new(status, "up") or history.new(status, "left_analog_up"):
        self.selecting_item_index = self.selecting_item_index - 1
        if self.selecting_item_index < 0:
          self.selecting_item_index = len(self.item_instances) - 1
        self.publishMenu(self.selecting_item_index)
        self.switchItem(self.selecting_item_index)
      elif status.circle and not latest.circle:
        #TODO action circle
        # rospy.loginfo("go_to_spot: " + self.prefix + self.current_item)
        self.spot_mixin.go_to_spot(self.prefix + self.current_item)
        self.command_pub.publish(self.circle_cmd)
        rospy.loginfo("circle")
      elif status.triangle and not latest.triangle:
        #TODO action triangle
        self.command_pub.publish(self.triangle_cmd)
        rospy.loginfo("triangle")
      elif status.square and not latest.square:
        #TODO action square
        self.command_pub.publish(self.square_cmd)
        rospy.loginfo("square")
      #elif status.cross and not latest.cross:
      #  
      else:
        self.publishMenu(self.selecting_item_index)

  def display(self, item):
    rospy.loginfo("Selecting spot: " + self.prefix + item)

  def enable(self):
    self.start()
    # TODO save the previous pose?
    self.selecting_item_index = 0
    self.switchItem(0)
    self.publishMenu(self.selecting_item_index, close=False)

  def disable(self):
    self.publishMenu(self.selecting_item_index, close=True)
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
