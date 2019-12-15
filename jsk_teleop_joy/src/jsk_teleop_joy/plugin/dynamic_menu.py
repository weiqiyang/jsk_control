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
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import tf.transformations

from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from jsk_rviz_plugins.msg import OverlayMenu
from status_history import StatusHistory

class DynamicMenu(JSKJoyPlugin):
  '''
Usage:
Up/Down: choose from menu

circle/square/triangle: publish cooperating command

Args:
frame_id [String, default: map]: frame_id of publishing pose
title [String, default: PoseList]: title of the menu
history [String, default: history]: rosparam name to load pose list
set_pose [String, default: set_pose]: topic name for publishing pose topic to set master
topic [String, default: dynamic_menu]: topic name to publish the menu
command [String, default: command]: topic name for publishing the command
triangle_cmd [String, default: DM_TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd [String, default: DM_CIRCLE_CMD]: command text when circle button is pressed
square_cmd [String, default: DM_SQUARE_CMD]: command text when suqare button is pressed
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
    self.current_item_index = 0
    self.selecting_item_index = 0
    self.title = self.getArg('title', 'PoseList')
    self.list_name = self.getArg('history', 'history')
    self.pose_pub = rospy.Publisher(self.getArg('set_pose', 'set_pose'),
                                      PoseStamped, queue_size=10)
    self.menu_pub = rospy.Publisher(self.getArg('topic', 'dynamic_menu'),
                                      OverlayMenu, queue_size=10)
    self.frame_id = self.getArg('frame_id', 'map')

    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    self.triangle_cmd = self.getArg('triangle_cmd', 'DM_TRIANGLE_CMD')
    self.square_cmd = self.getArg('square_cmd', 'DM_SQUARE_CMD')
    self.circle_cmd = self.getArg('circle_cmd', 'DM_CIRCLE_CMD')

    self.loadItems()
    self.start()

  def loadItems(self):
    if rospy.has_param(self.list_name):
      self.item_instances = rospy.get_param(self.list_name)
    else:
      rospy.set_param(self.list_name, [])
      self.item_instances = []

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
      rospy.logfatal('no valid history items are loaded')
      return False
    self.current_item = self.item_instances[0]
    return True

  def publishMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = [i[0] for i in self.item_instances]
    menu.current_index = index
    menu.title = "PoseList"
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
    rospy.loginfo("Load pose: " + str(item))
    pose = PoseStamped()
    pose.header.frame_id = self.frame_id
    pose.header.stamp = rospy.Time(0.0)
    pose.pose.position.x = item[1]
    pose.pose.position.y = item[2]
    pose.pose.position.z = item[3]
    pose.pose.orientation.x = item[4]
    pose.pose.orientation.y = item[5]
    pose.pose.orientation.z = item[6]
    pose.pose.orientation.w = item[7]
    self.pose_pub.publish(pose)

  def enable(self):
    self.loadItems()
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
