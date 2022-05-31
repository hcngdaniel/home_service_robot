#!/usr/bin/env python3
import rospy
import actionlib
import time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalStatusArray


class Navigation:
    def __init__(self, frame_id="map"):
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
        
        self.frame_id = frame_id
        self.init_pose = PoseWithCovarianceStamped()
        self.current_pose = PoseWithCovarianceStamped()
        self.goal = MoveBaseGoal()
        self.last_clicked_point = PointStamped()
        self.last_goal_pose = PoseStamped()
        self.status = GoalStatusArray()
        self.status_code = 0
        self.status_text = ""
        
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=self.__amcl_pose_callback)
        rospy.Subscriber("/clicked_point", PointStamped, callback=self.__clicked_point_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback=self.__goal_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, callback=self.__status_callback)
        
        self.__init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    
    def shutdown(self):
        self.move_base.cancel_goal()
    
    def __amcl_pose_callback(self, msg):
        self.current_pose = msg
    
    def __clicked_point_callback(self, msg):
        self.last_clicked_point = msg
    
    def __goal_callback(self, msg):
        self.last_goal_pose = msg
    
    def __status_callback(self, msg):
        self.status = msg
        if len(msg.status_list) > 0:
            self.status_code = msg.status_list[-1].status
            self.status_text = msg.status_list[-1].text
    
    def set_init_pose_in_rviz(self):
        self.init_pose = PoseWithCovarianceStamped()
        rospy.loginfo("Waiting for the robot's initial pose...")
        rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        rospy.loginfo("Set initial pose finished.")
        return self.init_pose.header.stamp != ""
    
    def get_clicked_point_in_rviz(self):
        rospy.loginfo("Waiting for rviz point...")
        rospy.wait_for_message("/clicked_point", PointStamped)
        rospy.loginfo("Read the rviz point")
        return self.last_clicked_point

    def get_clicked_pose_in_rviz(self):
        point = self.get_clicked_point_in_rviz()
        return self.point_to_pose(point.point.x, point.point.y, point.point.z)
    
    def set_goal_in_rviz(self):
        self.last_goal_pose = PoseStamped()
        rospy.loginfo("Waiting for rviz goal...")
        rospy.wait_for_message('move_base_simple/goal', PoseStamped)
        rospy.loginfo("Set goal finished")
        time.sleep(1)
        return self.last_goal_pose.header.stamp != ""

    @classmethod
    def point_to_pose(cls, x, y, a):
        pose = Pose()
        q = quaternion_from_euler(0.0, 0.0, a)
        q = Quaternion(q[0], q[1], q[2], q[3])
        pose.position.x = x
        pose.position.y = y
        pose.orientation = q
        return pose

    @classmethod
    def pose_to_point(cls, pose: Pose):
        x, y = pose.position.x, pose.position.y
        _, _, a = euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return x, y, a

    @classmethod
    def zw_to_a(cls, z, w):
        _, _, a = euler_from_quaternion([0, 0, z, w])
        return a

    @classmethod
    def Pose(cls, x, y, z, rx, ry, rz, rw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = rx
        pose.orientation.y = ry
        pose.orientation.z = rz
        pose.orientation.w = rw
        return pose
    
    def set_init_pose(self, x, y, a):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.point_to_pose(x, y, a)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        self.__init_pub.publish(msg)

    def move_to_pose(self, goal):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = self.frame_id
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = goal
        self.move_base.send_goal(self.goal)
        time.sleep(1)
        success = self.move_base.wait_for_result(rospy.Duration(300))
        return success
    
    def move_to(self, x, y, a):
        goal = self.point_to_pose(x, y, a)
        self.move_to_pose(goal)
        while not rospy.is_shutdown():
            if self.status_code == 0:
                pass
            if self.status_code == 1:
                pass
            if self.status_code == 3:
                break
            if self.status_code == 4:
                pass
            rospy.Rate(20).sleep()
