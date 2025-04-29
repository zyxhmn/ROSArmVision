#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory_command():
    rospy.init_node('direct_joint_command')
    
    # 创建Action客户端
    client = actionlib.SimpleActionClient(
        '/effort_joint_trajectory_controller/follow_joint_trajectory', 
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("等待控制器连接...")
    client.wait_for_server()
    rospy.loginfo("控制器已连接!")
    
    # 创建轨迹消息
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]
    
    # 添加一个轨迹点
    point = JointTrajectoryPoint()
    # 设置目标关节位置 - 只移动第一个关节
    point.positions = [0.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]
    point.time_from_start = rospy.Duration(2.0)  # 2秒后到达
    
    trajectory.points.append(point)
    
    # 创建并发送目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    
    rospy.loginfo("发送轨迹命令...")
    client.send_goal(goal)
    
    # 等待执行完成
    client.wait_for_result(rospy.Duration(10.0))
    result = client.get_result()
    
    if result:
        rospy.loginfo("轨迹执行完成!")
    else:
        rospy.logerr("轨迹执行失败或超时!")

if __name__ == '__main__':
    try:
        send_trajectory_command()
    except rospy.ROSInterruptException:
        pass