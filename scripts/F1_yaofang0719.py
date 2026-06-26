#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry  # 引入里程计消息
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String     # 引入String类型，接收视觉节点的字符串结果
from std_srvs.srv import Empty
import os
import random
import socket

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
     
        quaternions = list()
        euler_angles = (pi/2,pi, pi/2,-1.357, -pi/2, -0.684+pi/2,-1.238,0,-pi,0)
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        waypoints = list()
        waypoints.append(Pose(Point(1.284, 2.160, 0), quaternions[0]))      #//C 1.484
        waypoints.append(Pose(Point(0.603, 2.620, 0), quaternions[1]))      #//A 0.675,2.393   0.65  0.703
        waypoints.append(Pose(Point(1.284, 3.000, 0), quaternions[2]))      # //B x 减小//1.35,2.899 B小一点1.40，3.171
        waypoints.append(Pose(Point(-1.075, 0.898, 0), quaternions[3]))     # //4(-0.830, 0.969, 0)
        waypoints.append(Pose(Point(-1.731, 1.296, 0), quaternions[4]))    # //3  -1.781 （-1.650, 1.232, 0)
        waypoints.append(Pose(Point(-1.080, 1.715, 0), quaternions[5]))    # //2  -1.140
        waypoints.append(Pose(Point(-1.734, 2.325, 0), quaternions[6]))    # //1  -1.834
        waypoints.append(Pose(Point(0.002, -0.056, 0), quaternions[7]))    #起点
        waypoints.append(Pose(Point(-0.496, 3.921, 0), quaternions[8]))  #答题区(识别板2) 靠近里侧（-0.704,3.963,0） （0）靠近内侧 y减小(-0.784,3.750,0.1)
        waypoints.append(Pose(Point(0.866, -0.002, 0), quaternions[9]))  #识别板1向前一点0.6 ()x:0.65 y:0.025

        self.count = 9  #状态
        self.windows_ABC = 0
        self.windows_A = 1
        self.windows_B = 1
        self.windows_C = 1
        self.windows_1234 = 3
        self.windows_count = 3
        
        # 里程计相关变量初始化
        self.current_v = 0.0  # 线速度
        self.current_x = 0.0  # X坐标
        self.current_y = 0.0  # Y坐标

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result,queue_size=10)     
        # 订阅里程计和视觉字符串结果话题
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.vision_sub = rospy.Subscriber('/vision_report', String, self.vision_callback, queue_size=10)
        
        self.ram_result = [0, 0, 1, 1, 2] 
        rospy.sleep(1)

        # 初始化 TCP 网络连接
        self.referee_ip = '192.168.5.2'  
        self.referee_port = 8888
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client.settimeout(2.0) 
        try:
            self.tcp_client.connect((self.referee_ip, self.referee_port))
            rospy.loginfo("[网络] 成功连接到裁判系统！")
        except Exception as e:
            rospy.logwarn("[网络] 裁判系统连接失败 (未开启或IP错误)，单机模式运行。错误: %s" % e)
            self.tcp_client = None

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")
        
        while(not rospy.is_shutdown()):
            if(self.count == 9):
                rospy.loginfo("从起点到识别区")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[9]     
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别区。。。。。。。。")
                    
                    self.send_referee_status(task_id=1, info="到达识别板1")
                    
                    self.count = 10
                    rospy.sleep(6)     
                else:
                    rospy.logwarn("识别区导航失败")
                    self.count = 9
                    rospy.sleep(2)

            elif(self.count == 10):
                rospy.loginfo('10101010101010')
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()

                if(self.windows_C == 1): 
                    goal.target_pose.pose = waypoints[0]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到c窗口中的")
                        self.send_referee_status(task_id=2, info="取药窗口C")
                        self.count = 11
                        self.clear_costmaps_service()   
                        rospy.sleep(1)
                        if ((self.windows_A == 0) and (self.windows_B == 0)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangC.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiC.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeC.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueC.wav')

                if(self.windows_A == 1):
                    goal.target_pose.pose = waypoints[1]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到a窗口中的")
                        self.send_referee_status(task_id=2, info="取药窗口A")
                        self.count = 11
                        self.clear_costmaps_service()   
                        rospy.sleep(1)
                        if ((self.windows_C == 0) and (self.windows_B == 0)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangA.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiA.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeA.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueA.wav')
                        elif ((self.windows_C == 1) and (self.windows_B == 0)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAC.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAC.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAC.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAC.wav')
                        self.clear_costmaps_service()  

                if(self.windows_B == 1):
                    goal.target_pose.pose = waypoints[2]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到b窗口中的")
                        self.send_referee_status(task_id=2, info="取药窗口B")
                        self.count = 11
                        self.clear_costmaps_service()   
                        rospy.sleep(1)
                        if ((self.windows_C == 1) and (self.windows_A == 1)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangABC.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiABC.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeABC.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueABC.wav')
                        elif ((self.windows_C == 0) and (self.windows_A == 1)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAB.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAB.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAB.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAB.wav')
                        elif ((self.windows_C == 1) and (self.windows_A == 0)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangBC.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiBC.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeBC.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueBC.wav')
                        elif ((self.windows_C == 0) and (self.windows_A == 0)):
                            if(self.windows_1234 == 3): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangB.wav')
                            if(self.windows_1234 == 2): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiB.wav')
                            if(self.windows_1234 == 1): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeB.wav')
                            if(self.windows_1234 == 0): os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueB.wav')
                self.count = 11

            elif(self.count == 11):
                rospy.loginfo("从配药区到答题区路上")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[8]     
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别板2。。。。。。。。")
                    
                    self.send_referee_status(task_id=3, info="到达识别板2")
                    
                    self.count = 12
                    self.clear_costmaps_service()   
                    # [预留接口] 以后这里接入从识别板2识别出来的“等待状态”
                    rospy.loginfo("化验区空闲，默认不等待") 
                    rospy.sleep(1)     

            elif(self.count == 12):
                rospy.loginfo("从答题区到数字区路上")
                goal = MoveBaseGoal() 
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[(6 - self.windows_1234)]
                if(self.move(goal) == True):
                    rospy.loginfo("到达数字区。。。。。。。。")   
                    
                    self.send_referee_status(task_id=4, info="化验区送药完成")                 
                    
                    self.count = 9 
                    self.clear_costmaps_service()   
                    rospy.sleep(1)     
                    
                    if self.windows_1234 == 3: 
                        rospy.loginfo("到达激素检验窗口")
                        if self.windows_count == 3: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu3.wav')
                        elif self.windows_count == 2: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu2.wav')
                        elif self.windows_count == 1: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu1.wav')
                    if self.windows_1234 == 2: 
                        rospy.loginfo("到达免疫检验窗口")
                        if self.windows_count == 3: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi3.wav')
                        elif self.windows_count == 2: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi2.wav')
                        elif self.windows_count == 1: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi1.wav')
                    if self.windows_1234 == 1: 
                        rospy.loginfo("到达体液检验窗口")
                        if self.windows_count == 3: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye3.wav')
                        elif self.windows_count == 2: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye2.wav')
                        elif self.windows_count == 1: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye1.wav')
                    if self.windows_1234 == 0: 
                        rospy.loginfo("到达血常规检验窗口")
                        if self.windows_count == 3: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui3.wav')
                        elif self.windows_count == 2: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui2.wav')
                        elif self.windows_count == 1: os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui1.wav')

    # === 里程计回调函数，实时更新坐标和速度 ===
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_v = msg.twist.twist.linear.x

    # === 视觉识别字符串回调，负责把结果发给裁判 ===
    def vision_callback(self, msg):
        # 如果当前正处于识别区域（count=10是因为识别完成后马上进入10状态准备去取药）
        # 这里用 count == 1