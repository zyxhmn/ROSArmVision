#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from geometry_msgs.msg import PointStamped
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from tf.transformations import quaternion_from_euler
import math
from franka_gripper.msg import StopAction, StopGoal

class PandaGraspNode:
    def __init__(self):
        rospy.init_node('panda_grasp_node', anonymous=True)

        # MoveIt 初始化
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        # 夹爪客户端
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.grasp_client.wait_for_server()
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper_client.wait_for_server()

        # 目标点
        self.target_position = None

        # 订阅视觉目标话题
        self.target_sub = rospy.Subscriber('/detected_blue_cube_world', PointStamped, self.target_callback)

        # 预定义放置点
        self.target_pose = geometry_msgs.msg.Pose()
        self.target_pose.position.x = 0.95
        self.target_pose.position.y = 1
        self.target_pose.position.z = 0.6
        q = quaternion_from_euler(-math.pi, 0, -math.pi/4)
        self.target_pose.orientation.x = q[0]
        self.target_pose.orientation.y = q[1]
        self.target_pose.orientation.z = q[2]
        self.target_pose.orientation.w = q[3]

        rospy.loginfo("Panda抓取节点初始化完成，等待视觉节点发布目标位置...")

    def target_callback(self, msg):
        self.target_position = msg.point
        rospy.loginfo("接收到目标位置: x=%.3f, y=%.3f, z=%.3f",
                      self.target_position.x,
                      self.target_position.y,
                      self.target_position.z)

        response = input("检测到目标，是否进行抓取? (y/n): ").strip().lower()
        if response == 'y':
            self.execute_grasp()
        else:
            rospy.loginfo("跳过当前目标")

    def open_gripper(self):
        rospy.loginfo("打开夹爪...")
        goal = MoveGoal(width=0.08, speed=0.05)
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        return self.gripper_client.get_result()

    def grasp_object(self, width=0.03, epsilon_inner=0.005, epsilon_outer=0.005, speed=0.1, force=10):
        rospy.loginfo("夹持物体...")
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        goal.speed = speed
        goal.force = force
        self.grasp_client.send_goal(goal)
        
        # 设置超时，不要无限等待
        success = self.grasp_client.wait_for_result(rospy.Duration(2.0))
        # if not success:
        #     rospy.logwarn("抓取动作超时，但这可能是正常的持续施力状态")
        
        # 无需调用stop，让夹爪持续施加力度
        return self.grasp_client.get_result()

    def stop_gripper(self):
        rospy.loginfo("停止夹爪施力...")
        stop_client = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)
        stop_client.wait_for_server(rospy.Duration(1.0))
        stop_client.send_goal(StopGoal())
        stop_client.wait_for_result(rospy.Duration(1.0))

    def go_to_home_position(self):
        rospy.loginfo("回到初始位置...")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.785398
        joint_goal[2] = 0
        joint_goal[3] = -2.35619
        joint_goal[4] = 0
        joint_goal[5] = 1.57079
        joint_goal[6] = 0.785398

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def move_to_pose(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        plan_success, plan, _, _ = self.move_group.plan()
        if plan_success:
            rospy.loginfo("规划成功，执行移动...")
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logerr("运动规划失败")
        self.move_group.clear_pose_targets()

    def execute_grasp(self):
        if self.target_position is None:
            rospy.logwarn("没有目标位置，无法执行抓取")
            return

        try:
            # 打开夹爪
            self.open_gripper()

            # 预抓取位置（目标点上方，稍微偏移）
            pre_grasp_pose = geometry_msgs.msg.Pose()
            pre_grasp_pose.position.x = self.target_position.x - 0.05
            pre_grasp_pose.position.y = self.target_position.y
            pre_grasp_pose.position.z = self.target_position.z + 0.4

            q = quaternion_from_euler(-math.pi, 0, -math.pi/4)
            pre_grasp_pose.orientation.x = q[0]
            pre_grasp_pose.orientation.y = q[1]
            pre_grasp_pose.orientation.z = q[2]
            pre_grasp_pose.orientation.w = q[3]

            # 抓取位置（目标点略高）
            grasp_pose = geometry_msgs.msg.Pose()
            grasp_pose.position.x = self.target_position.x
            grasp_pose.position.y = self.target_position.y + 0.02
            grasp_pose.position.z = self.target_position.z + 0.07
            grasp_pose.orientation = pre_grasp_pose.orientation

            rospy.loginfo("移动到抓取位置（慢速）...")
            self.move_group.set_max_velocity_scaling_factor(0.1)
            self.move_to_pose(grasp_pose)
            self.move_group.set_max_velocity_scaling_factor(0.2)

            # 抓取动作
            self.grasp_object(width=0.04, force=100)

            # 抬升物体
            lift_pose = geometry_msgs.msg.Pose()
            lift_pose.position.x = grasp_pose.position.x
            lift_pose.position.y = grasp_pose.position.y
            lift_pose.position.z = grasp_pose.position.z + 0.2
            lift_pose.orientation = grasp_pose.orientation

            rospy.loginfo("抬升物体...")
            # 如果需要可取消注释下一行执行抬升动作
            # self.move_to_pose(lift_pose)

            # 移动到放置点
            rospy.loginfo("移动到放置位置...")
            # self.grasp_object(width=0.05, force=5)
            self.move_to_pose(self.target_pose)
            # self.grasp_object(width=0.05, force=5)
            rospy.loginfo("抓取并放置操作完成")

            # 释放物体
            self.open_gripper()
            # 返回初始位置
            rospy.loginfo("返回初始位置...")
            self.go_to_home_position()

        except Exception as e:
            rospy.logerr("抓取过程中出错: %s", str(e))


def main():
    try:
        node = PandaGraspNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
