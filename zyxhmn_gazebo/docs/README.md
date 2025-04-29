## ROSArmVision

## 环境

系统:ubuntu20

ros:noetic

gazebo:11

## 安装









roslaunch franka_gazebo panda.launch  world:=/home/sea/zhoudapao_ws/src/zyxhmn_gazebo/worlds/room.world



roslaunch panda_moveit_config demo_gazebo.launch world:=/home/sea/zhoudapao_ws/src/zyxhmn_gazebo/worlds/room.world



rosrun image_view image_view image:=/detected_lemon/image



blue :0.209639 0.258602 0.398231

green:0.008505 0.257563 0.398240

bin: 0.506 1.406 0





```bash
我有一个ros项目需要视觉处理，识别gazebo环境中的蓝色方块，并给出相对world的坐标，相机的姿态为<origin xyz="0 0.85 1.5" rpy="0 1.5708 0"/>，参数为<xacro:macro name="gazebo_d435i" params=" reference_link
                                            camera_name:=camera
                                            topics_ns:=camera

                                            depth_optical_frame
                                            color_optical_frame
                                            infrared1_optical_frame
                                            infrared2_optical_frame
                                            accel_optical_frame
                                            gyro_optical_frame

                                            visualize:=false
                                            align_depth:=false
                                            enable_pointCloud:=false

                                            unite_imu_method:=false
                                            accel_fps:=250
                                            gyro_fps:=400

                                            clip_distance:=-1.0
                                            depth_width:=1280
                                            depth_height:=720
                                            depth_fps:=30

                                            infra_width:=640
                                            infra_height:=480
                                            infra_fps:=30

                                            color_width:=1920
                                            color_height:=1080
                                            color_fps:=30
                                            ">
```

