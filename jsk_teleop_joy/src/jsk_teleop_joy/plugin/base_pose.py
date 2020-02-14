from joy_rviz_view_controller import RVizViewController

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf
import rospy
import numpy
import math
import time

def signed_square(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign

class BasePose(RVizViewController):
    '''
Usage:
Left Analog x/y: translate x/y
L1/R1: rotate yaw
square: move faster

Right Analog x/y: yaw/pitch of camera position (see parent class, RVizViewController)
R3(Right Analog button): suppressing buttons/sticks for controlling pose
   R3 + L2 + R2: enable follow view mode

circle/cross/triangle: publish corresponding command

Args:
publish_pose [Boolean, default: True]: publish pose or not
frame_id [String, default: base_footprint]: frame_id of publishing pose, overwritten by parameter ~frame_id
map_frame_id [String, default: map]: frame_id of map
goal [String, default: base]: topic name for publishing base pose
target_pose [String, default: target_pose]: topic name to pubish current pose when button is pressed
set_base [String, default: set_base]: topic name for setting base pose by topic
z [float, default: 0.0]: initial value for z, overwritten by arg set_base
command [String, default: command]: topic name for publishing the command
triangle_cmd [String, default: BP_TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd [String, default: BP_CIRCLE_CMD]: command text when circle button is pressed
cross_cmd [String, default: BP_CROSS_CMD]: command text when cross button is pressed
save_key [Int32, default: -1]: key used to save the pose. -1: do not save; 0: circle; 1: triangle; 2: cross
pose_list [String, default: history]: rosparam name to save pose list
list_length [Int32, default: 5]: maximum length of pose_list. will overwrite the oldest data when the list is full
    '''
    CIRCLE = 0
    TRIANGLE = 1
    CROSS = 2

    def __init__(self, name, args):
        RVizViewController.__init__(self, name, args)
        self.publish_pose = self.getArg('publish_pose', True)

        self.pre_pose = PoseStamped()
        #self.pre_pose.header.stamp = rospy.Time(0)
        self.pre_pose.pose.orientation.w = 1
        self.z = self.getArg('z', 0.0)
        self.pre_pose.pose.position.z = self.z
        self.prev_time = rospy.Time.from_sec(time.time())
        self.frame_id = self.getArg('frame_id', 'base_footprint')
        self.map_frame_id = self.getArg('map_frame_id', 'map')
        self.tf_listener = tf.TransformListener()
        self.count = 0

        self.goal_type = self.getArg('goal_type', 'PoseStamped')
        self.target_pub = rospy.Publisher(self.getArg('target_pose', 'target_pose'),
                                          PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                           String, queue_size=1)

        if self.publish_pose:
            self.pose_pub = rospy.Publisher(self.getArg('goal', 'joy_base_pose'),
                                            PoseStamped, queue_size=1)
        self.triangle_cmd = self.getArg('triangle_cmd', 'BP_TRIANGLE_CMD')
        self.cross_cmd = self.getArg('cross_cmd', 'BP_CROSS_CMD')
        self.circle_cmd = self.getArg('circle_cmd', 'BP_CIRCLE_CMD')
        self.supportFollowView(True)
        self.pose_sub = rospy.Subscriber(self.getArg('set_base', 'set_base'),
                                         PoseStamped, self.set_pose_cb, queue_size=1)

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

        self.save_key = self.getArg('save_key', -1)
        if self.save_key > -1:
            self.pose_list_name = self.getArg('pose_list', 'history')
            self.list_length = self.getArg('list_length', 5)
            self.init_save_list()
        rospy.loginfo("End loading base_pose")

    def set_pose_cb(self, pose):
        pose.header.stamp = rospy.Time(0)
        self.pre_pose = self.tf_listener.transformPose(self.map_frame_id, pose)
        if self.publish_pose:
            self.pose_pub.publish(self.pre_pose)

    def publish_pose_command(self, pose, command):
        rospy.loginfo("Publishing command: {}, pose: {}".format(command, pose.pose))
        self.pose_pub.publish(pose)
        self.command_pub.publish(command)

    def publish_goal_command(self, pose, command):
        rospy.loginfo("Publishing goal: {}, goal: {}".format(command, pose.pose))
        pose.header.stamp = rospy.Time.now()
        self.target_pub.publish(pose)
        self.command_pub.publish(command)

    def init_save_list(self):
        if not rospy.has_param(self.pose_list_name) or len(rospy.get_param(self.pose_list_name)) < 1:
            x = self.pre_pose.pose.position.x
            if type(x) == type(numpy.float64()):
                x = x.item()
            y = self.pre_pose.pose.position.y
            if type(y) == type(numpy.float64()):
                y = y.item()
            rospy.set_param(self.pose_list_name, [["Initial", x,y,self.z,0,0,0,0]])
        self.item_instances = rospy.get_param(self.pose_list_name)
        if len(self.item_instances) > self.list_length:
            self.item_instances = self.item_instances[0:self.list_length]

    def save_current_pose(self, pose):
        tag = '[' + str(self.count) + '] ' + self.pose_list_name
        self.count += 1
        pose_arr = [0] * 8
        pose_arr[0] = tag
        pose_arr[1] = pose.pose.position.x
        pose_arr[2] = pose.pose.position.y
        pose_arr[3] = self.z
        pose_arr[4] = pose.pose.orientation.x
        pose_arr[5] = pose.pose.orientation.y
        pose_arr[6] = pose.pose.orientation.z
        pose_arr[7] = pose.pose.orientation.w
        for i in range(1,8):
            if type(pose_arr[i]) == type(numpy.float64()):
                pose_arr[i] = pose_arr[i].item()
        new_list = [pose_arr]
        new_list.extend(self.item_instances)
        if len(new_list) > self.list_length:
            new_list = new_list[0:self.list_length]
        self.item_instances = new_list
        rospy.set_param(self.pose_list_name, self.item_instances)
        rospy.loginfo("Saved base pose: " + str(pose_arr))

    def joyCB(self, status, history):
        pre_pose = self.pre_pose
        if history.length() > 0:
            latest = history.latest()
            if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
                self.followView(not self.followView())
        if self.control_view:
            RVizViewController.joyCB(self, status, history)
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
                self.publish_goal_command(new_pose, self.circle_cmd)
                if self.save_key == self.CIRCLE:
                    self.save_current_pose(new_pose)
            if status.triangle and not latest.triangle:
                self.publish_pose_command(new_pose, self.triangle_cmd)
                if self.save_key == self.TRIANGLE:
                    self.save_current_pose(new_pose)
            if status.cross and not latest.cross:
                self.publish_pose_command(new_pose, self.cross_cmd)
                if self.save_key == self.CROSS:
                    self.save_current_pose(new_pose)

        # publish at 10hz
        if self.publish_pose:
            now = rospy.Time.from_sec(time.time())
            # placement.time_from_start = now - self.prev_time
            if (now - self.prev_time).to_sec() > 1 / 30.0:
                self.pose_pub.publish(new_pose)
                self.prev_time = now

        self.pre_pose = new_pose
