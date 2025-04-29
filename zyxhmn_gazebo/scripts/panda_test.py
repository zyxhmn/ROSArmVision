#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import time

class PandaArmTest:
    def __init__(self):
        # 初始化节点
        rospy.init_node('panda_arm_test', anonymous=True)
        
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 获取机器人和规划组
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 使用panda_arm规划组
        self.group_name = "panda_arm"
        self.arm_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 设置参数
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_max_velocity_scaling_factor(0.1)  # 慢速移动
        self.arm_group.set_max_acceleration_scaling_factor(0.1)  # 慢速加速
        
        # 显示规划轨迹
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)
        
        # 打印信息
        self.print_robot_info()
        
    def print_robot_info(self):
        """打印机器人信息"""
        rospy.loginfo("=== 机器人信息 ===")
        rospy.loginfo("规划框架: %s", self.arm_group.get_planning_frame())
        rospy.loginfo("末端执行器链接: %s", self.arm_group.get_end_effector_link())
        rospy.loginfo("可用规划组: %s", self.robot.get_group_names())
        
        # 获取当前关节位置和末端位姿
        joints = self.arm_group.get_current_joint_values()
        pose = self.arm_group.get_current_pose().pose
        
        rospy.loginfo("当前关节位置: %s", str(joints))
        rospy.loginfo("当前末端位姿: 位置 [%f, %f, %f], 方向 [%f, %f, %f, %f]", 
                      pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        
    def move_to_home(self):
        """移动到初始位置"""
        rospy.loginfo("尝试移动到初始位置...")
        
        # 预定义的初始关节角度
        joint_goal = [0, -pi/4, 0, -pi/2, 0, pi/3, 0]
        
        # 执行移动
        success = self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("成功移动到初始位置")
        else:
            rospy.logerr("移动到初始位置失败")
        
        return success
    
    def move_joint_by_increment(self, joint_index, increment):
        """增量移动单个关节"""
        if joint_index < 0 or joint_index >= 7:
            rospy.logerr("无效的关节索引: %d (应为0-6)", joint_index)
            return False
        
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[joint_index] += increment
        
        rospy.loginfo("尝试移动关节 %d 增量 %f...", joint_index, increment)
        success = self.arm_group.go(joint_values, wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("成功移动关节")
        else:
            rospy.logerr("移动关节失败")
        
        return success
    
    def move_to_cartesian_pose(self, x, y, z):
        """移动到笛卡尔空间中的位置"""
        current_pose = self.arm_group.get_current_pose().pose
        
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation = current_pose.orientation  # 保持当前方向
        
        rospy.loginfo("尝试移动到位置 [%f, %f, %f]...", x, y, z)
        self.arm_group.set_pose_target(target_pose)
        
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("成功移动到目标位置")
        else:
            rospy.logerr("移动到目标位置失败")
            
            # 尝试笛卡尔路径规划
            rospy.loginfo("尝试使用笛卡尔路径规划...")
            waypoints = []
            waypoints.append(target_pose)
            
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
            
            if fraction > 0.5:  # 如果计划覆盖了至少50%的路径
                success = self.arm_group.execute(plan, wait=True)
                if success:
                    rospy.loginfo("笛卡尔路径执行成功")
                else:
                    rospy.logerr("笛卡尔路径执行失败")
            else:
                rospy.logerr("笛卡尔路径规划覆盖率低: %f", fraction)
        
        return success
    
    def run_test_sequence(self):
        """运行一系列测试动作"""
        rospy.loginfo("开始运行测试序列...")
        
        # 测试1: 移动到初始位置
        if not self.move_to_home():
            rospy.logwarn("初始位置测试失败，继续下一个测试...")
        
        time.sleep(1)
        
        # 测试2: 增量移动关节
        for i in range(7):
            self.move_joint_by_increment(i, 0.1)
            time.sleep(1)
            self.move_joint_by_increment(i, -0.1)  # 恢复原位
            time.sleep(1)
        
        # 测试3: 笛卡尔空间移动
        current_pose = self.arm_group.get_current_pose().pose
        
        # 尝试在当前位置的基础上向前移动10cm
        target_x = current_pose.position.x + 0.1
        target_y = current_pose.position.y
        target_z = current_pose.position.z
        
        self.move_to_cartesian_pose(target_x, target_y, target_z)
        time.sleep(1)
        
        # 返回初始位置
        self.move_to_home()
        
        rospy.loginfo("测试序列完成")

def main():
    try:
        # 创建测试对象
        test = PandaArmTest()
        
        # 等待用户确认
        input("\n按Enter键开始测试序列...")
        
        # 运行测试序列
        test.run_test_sequence()
        
        # 提示用户
        rospy.loginfo("测试完成。按Ctrl+C退出。")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()