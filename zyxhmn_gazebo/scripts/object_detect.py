#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
# import image_geometry # 虽然此方法不需要，但保留以备不时之需

class BlueCubeDetectorAligned:
    def __init__(self):
        rospy.init_node('blue_cube_detector_aligned', anonymous=True)

        # 定义蓝色在HSV颜色空间中的范围
        self.hsv_lower_blue = np.array([100, 150, 50])
        self.hsv_upper_blue = np.array([140, 255, 255])
        # 最小轮廓面积，基于原始彩色图像尺寸
        self.min_contour_area = 500

        # 初始化CvBridge，用于ROS图像消息与OpenCV图像之间的转换
        self.bridge = CvBridge()
        # 初始化TF2缓冲区和监听器，用于坐标变换
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- 相机内参 ---
        # 需要彩色相机内参，用于使用对齐深度时的重建
        # 从/D435i_camera/color/camera_info获取这些参数
        self.color_K = np.array([1386.4139404296875, 0.0, 960.0,
                                 0.0, 1386.4139404296875, 540.0,
                                 0.0, 0.0, 1.0]).reshape((3, 3))
        self.color_fx = self.color_K[0, 0]
        self.color_fy = self.color_K[1, 1]
        self.color_cx = self.color_K[0, 2]
        self.color_cy = self.color_K[1, 2]
        self.color_width = 1920
        self.color_height = 1080

        # 存储彩色图像的frame_id（重要！）- 从彩色相机信息头中获取
        self.color_frame_id = "D435i_camera_color_optical_frame"  # 确保这与你的系统匹配
        # 可通过以下命令检查：rostopic echo /D435i_camera/color/camera_info -h

        # --- 订阅者 ---
        # 订阅彩色图像和对齐的深度图像
        color_sub = message_filters.Subscriber('/D435i_camera/color/image_raw', Image)
        # *** 如果你的话题名称不同，请更改此话题名称 ***
        aligned_depth_sub = message_filters.Subscriber('/D435i_camera/aligned_depth_to_color/image_raw', Image)

        # 使用ApproximateTimeSynchronizer同步彩色和深度图像
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, aligned_depth_sub], queue_size=10, slop=0.1  # 如有需要可调整slop
        )
        self.ts.registerCallback(self.image_callback)

        rospy.loginfo("蓝色立方体检测节点已初始化（使用对齐深度）。等待图像中...")
        rospy.loginfo(f"预期对齐深度话题：{aligned_depth_sub.name}")
        rospy.loginfo(f"使用彩色帧ID进行TF变换：{self.color_frame_id}")

        # --- 发布者 ---
        # 发布检测到的蓝色立方体在世界坐标系中的位置
        self.world_point_pub = rospy.Publisher('/detected_blue_cube_world', PointStamped, queue_size=1)
        # 发布用于调试的彩色图像
        self.debug_color_pub = rospy.Publisher('/detection_debug_color', Image, queue_size=1)
        # 可选：发布对齐深度图像以供验证
        self.debug_aligned_depth_pub = rospy.Publisher('/detection_debug_aligned_depth', Image, queue_size=1)

    def image_callback(self, color_msg, aligned_depth_msg):
        # 验证时间戳差异是否过大（ sanity check）
        time_diff = abs(color_msg.header.stamp - aligned_depth_msg.header.stamp)
        if time_diff > rospy.Duration(0.05):  # 如果差异大于50ms则发出警告
            rospy.logwarn_throttle(5.0, f"彩色图像和对齐深度图像的时间戳差异较大：{time_diff.to_sec():.4f}s")

        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            # 对齐深度图像通常为16UC1（单位：毫米）
            cv_aligned_depth_image = self.bridge.imgmsg_to_cv2(aligned_depth_msg, "passthrough")

            if cv_aligned_depth_image.dtype == np.uint16:
                # 将深度值从毫米转换为米
                cv_aligned_depth_meters = cv_aligned_depth_image.astype(np.float32) / 1000.0
            elif cv_aligned_depth_image.dtype == np.float32:
                # 不太可能出现，但仍处理这种情况
                cv_aligned_depth_meters = cv_aligned_depth_image
            else:
                rospy.logwarn_throttle(5, f"意外的对齐深度图像类型：{cv_aligned_depth_image.dtype}")
                return

            # 创建用于调试的彩色图像副本
            debug_color_img = cv_color_image.copy()

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge错误：{e}")
            return

        # 1. 在原始彩色图像中检测蓝色立方体
        hsv_image = cv2.cvtColor(cv_color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.hsv_lower_blue, self.hsv_upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            # 如果没有轮廓，发布原始彩色图像
            try:
                self.debug_color_pub.publish(self.bridge.cv2_to_imgmsg(debug_color_img, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(f"发布调试图像时CvBridge错误：{e}")
            return

        # 找到面积最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < self.min_contour_area:
            return

        # 在原始彩色图像坐标系中计算中心点 (uc, vc)
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return
        uc = int(M["m10"] / M["m00"])
        vc = int(M["m01"] / M["m00"])

        # 限制坐标在图像范围内（重要！）
        uc = max(0, min(uc, self.color_width - 1))
        vc = max(0, min(vc, self.color_height - 1))

        # 在彩色调试图像上绘制轮廓和中心点
        cv2.drawContours(debug_color_img, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(debug_color_img, (uc, vc), 7, (0, 0, 255), -1)

        # 2. 直接从对齐深度图像中获取深度值，使用 (uc, vc)
        depth_raw = cv_aligned_depth_meters[vc, uc]  # 直接索引：vc=行，uc=列

        rospy.loginfo(f"--- 处理轮廓 ---")
        rospy.loginfo(f"彩色中心点 (uc, vc): ({uc}, {vc})")
        rospy.loginfo(f"在 ({uc},{vc}) 处的对齐图像原始深度值：{depth_raw:.4f} 米")

        depth = depth_raw
        # 检查深度值是否无效（对齐深度中通常为0），并尝试邻域值
        if depth <= 0.01 or np.isnan(depth):
            rospy.logwarn(f"在 ({uc},{vc}) 处的对齐深度图中深度值无效 ({depth_raw:.4f})。尝试邻域值。")
            radius = 5
            # 在对齐深度图像中搜索
            min_r, max_r = max(0, vc-radius), min(self.color_height, vc+radius+1)
            min_c, max_c = max(0, uc-radius), min(self.color_width, uc+radius+1)
            neighbors = cv_aligned_depth_meters[min_r:max_r, min_c:max_c]
            valid_depths = neighbors[(neighbors > 0.1) & (neighbors < 10.0) & ~np.isnan(neighbors)]
            if valid_depths.size > 0:
                depth = np.median(valid_depths)
                rospy.logwarn(f"使用中值深度 {depth:.4f}米，来自 {valid_depths.size} 个邻域点。")
            else:
                rospy.logwarn(f"在 ({uc},{vc}) 处的对齐深度图中未找到有效邻域点。跳过该点。")
                # 在返回前发布调试图像
                try:
                    self.debug_color_pub.publish(self.bridge.cv2_to_imgmsg(debug_color_img, "bgr8"))
                except CvBridgeError as e:
                    rospy.logerr(f"发布调试图像时CvBridge错误：{e}")
                return

        # 可选：发布对齐深度图像以供可视化
        try:
            # 为了显示进行归一化
            depth_display = cv_aligned_depth_meters.copy()
            depth_display[np.isnan(depth_display)] = 0
            depth_display = np.clip(depth_display, 0.1, 3.0)
            depth_display = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_display_bgr = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)
            # 在对齐深度图像上绘制查找点 (uc, vc)
            cv2.circle(depth_display_bgr, (uc, vc), 7, (0, 0, 255), -1)  # 在查找点处绘制红色圆圈
            self.debug_aligned_depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_display_bgr, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"发布对齐深度调试图像时CvBridge错误：{e}")

        # 3. 在彩色相机光学坐标系中计算3D点
        # *** 使用彩色相机内参 ***
        cam_x = (uc - self.color_cx) * depth / self.color_fx
        cam_y = (vc - self.color_cy) * depth / self.color_fy
        cam_z = depth

        point_camera = PointStamped()
        point_camera.header.stamp = aligned_depth_msg.header.stamp  # 使用对齐深度的时间戳
        # *** 使用彩色帧ID ***
        point_camera.header.frame_id = self.color_frame_id
        point_camera.point.x = cam_x
        point_camera.point.y = cam_y
        point_camera.point.z = cam_z

        rospy.loginfo(f"在 {point_camera.header.frame_id} 中的点："
                      f"x={cam_x:.3f}, y={cam_y:.3f}, z={cam_z:.3f}")

        # 4. 将点转换到世界坐标系
        try:
            target_frame = 'world'
            source_frame = point_camera.header.frame_id  # 现在是彩色帧

            # 在尝试之前检查变换是否存在
            if not self.tf_buffer.can_transform(target_frame, source_frame, point_camera.header.stamp, rospy.Duration(0.1)):
                rospy.logwarn_throttle(2.0, f"无法从 '{source_frame}' 转换到 '{target_frame}'。TF不可用。")
                return

            point_world = self.tf_buffer.transform(point_camera, target_frame, timeout=rospy.Duration(0.1))

            rospy.loginfo(f">>> 蓝色立方体世界坐标："
                          f"x={point_world.point.x:.3f}, "
                          f"y={point_world.point.y:.3f}, "
                          f"z={point_world.point.z:.3f} <<<")
            self.world_point_pub.publish(point_world)

            # 在彩色图像上绘制世界坐标
            coord_text = f"W:({point_world.point.x:.2f},{point_world.point.y:.2f},{point_world.point.z:.2f})"
            cv2.putText(debug_color_img, coord_text, (uc + 10, vc + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)  # 青色文本

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"从 '{source_frame}' 到 '{target_frame}' 的TF变换错误：{e}")

        # 发布彩色调试图像
        try:
            color_debug_msg = self.bridge.cv2_to_imgmsg(debug_color_img, "bgr8")
            color_debug_msg.header = color_msg.header
            self.debug_color_pub.publish(color_debug_msg)
        except CvBridgeError as e:
            rospy.logerr(f"发布彩色调试图像时CvBridge错误：{e}")

        # 可选：本地显示
        # cv2.imshow("Color Debug", debug_color_img)
        # if 'depth_display_bgr' in locals():
        #     cv2.imshow("Aligned Depth Debug", depth_display_bgr)
        # cv2.waitKey(1)

def main():
    try:
        # 移除了wait_for_message块

        # 直接初始化检测器
        rospy.loginfo("初始化检测器，等待同步话题...")
        detector = BlueCubeDetectorAligned()
        rospy.spin()  # 节点将持续运行直到消息到达

    except rospy.ROSInterruptException:
        pass
    finally:
        # 即使检测器未完全使用，也可能会调用此代码，这没问题
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
