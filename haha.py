#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import subprocess
import difflib
import rospy
import serial
import numpy 
# from tf_conversions import transformations   
from geometry_msgs.msg import *
from std_msgs.msg import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import actionlib
from cv_bridge import CvBridge
import sys
import cv2
from cv_bridge import *
from sensor_msgs.msg import *
import math
from copy import deepcopy  
from math import pi   
import base64
from nav_msgs.msg import *
import sys
import time
import tf2_ros
import tf
from sensor_msgs.msg import Image
from playsound import playsound
from darknet_ros_msgs.msg import BoundingBoxes
#from aip import AipOcr
import re
import easyocr
from collections import OrderedDict
import json
import paho.mqtt.client as mqtt



cam0_path  = '/home/hgauto/webviz/dist/sensor/' 
class HG_API:
    def __init__(self):

        self.send_detect_img = None
        self.nav_data_=0
        self.power_data=0

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.nav=0
        self.detect_obj_img =None
        self.bridge = CvBridge()
        self.current_x=0
        self.kp = 1
        self.tolerance = 0.01
        self.goal_x=0
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.mqtt_read_buf = ""


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x # 当前位置

    def move(self,x,n):
        cmd = Twist()
        while x!=0 and n!=0:
            r=n*3.1415926535/180
            #车左转
            if x>0 and n>0:
                cmd.angular.z = r
                if x>1:
                    time1=x/0.2+10
                else:
                    time1=x/0.2
                cmd.linear.x=0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                cmd.angular.z = 0
                self.cmd_pub.publish(cmd)
                print("Done!!!")
                return True

            #车右转
            elif x>0 and n<0:
                cmd.angular.z =r
                if x>1:
                    time1=x/0.2+10
                else:
                    time1=x/0.2
                cmd.linear.x=0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                cmd.angular.z = 0
                self.cmd_pub.publish(cmd)
                print("Done!!!")
                return True
            #车后左转
            elif x<0 and n>0:
                cmd.angular.z = r
                if x<-1:
                    time1=-x/0.2+10
                else:
                    time1=-x/0.2
                cmd.linear.x=-0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                cmd.angular.z = 0
                self.cmd_pub.publish(cmd)
                print("Done!!!")
                return True
            #车后右转
            elif x<0 and n<0:
                cmd.angular.z = r
                if x<-1:
                    time1=-x/0.2+10
                else:
                    time1=-x/0.2
                cmd.linear.x=-0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                cmd.angular.z = 0
                self.cmd_pub.publish(cmd)
                print("Done!!!")
                return True

        #只前进后退
        while x!=0 and n==0:
            #车前行
            if x>0 :
                time1=x/0.2+10
                print(time1)
                cmd.linear.x=0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                print("Done!!!")
                return True
            elif x<0:
                time1=-x/0.2+10
                print(time1)
                cmd.linear.x=-0.2

                for i in range(0,int(time1*10),10):
                    self.cmd_pub.publish(cmd)
                    print(i)
                    time.sleep(0.5)
                print("Done!!!")
                return True


    # 输入目标点，到达指定的目标点的接口函数
    # 输入参数为：map坐标系下的小车的期望位置target_x,target_y,target_z以及期望姿态                   target_qx,target_qy,target_qz,target_qw
    # api.Target_point(1.23,1.0,0,0,0,0.666,0.74)
    # 需要启动底盘和导航功能
    def target_point(self,target_x,target_y,target_z,target_qx,target_qy,target_qz,target_qw):
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
        # 等待连接服务器，5s等待时间限制 
        while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
            rospy.loginfo("Connected to move base server")  
        # 设定目标点  
        target_1 = Pose(Point(target_x, target_y, target_z), Quaternion(target_qx, target_qy,target_qz,target_qw))  
        goal_1 = MoveBaseGoal()  
        goal_1.target_pose.pose = target_1  
        goal_1.target_pose.header.frame_id = 'map'  
        goal_1.target_pose.header.stamp = rospy.Time.now()  
        rospy.loginfo("Going to: " + str(target_1))  
        # 向目标进发  
        move_base.send_goal(goal_1)  
        # 五分钟时间限制  
        finished_within_time = move_base.wait_for_result(rospy.Duration(120))   

        # 查看是否成功到达  
        if not finished_within_time:  
            move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
            return False
        else:  
            state = move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:
                self.nav+=1  
                rospy.loginfo("目标点：(%.4f,%.4f,%.4f)成功到达！",target_x,target_y,target_z)
                return True
            else:  
                rospy.loginfo("目标点：(%.4f,%.4f,%.4f)无法到达！",target_x,target_y,target_z)
                return False
        '''
            获取小车当前的位置和姿态
            返回参数：位移（x,y,z）以及四元数的（x,y,z,w）
        '''
    def get_position(self):
        time.sleep(1)
        try:
            tfs = self.buffer.lookup_transform("map","base_link",rospy.Time(0))
            # euler = transformations.euler_from_quaternion([tfs.transform.rotation.x,tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
            return [tfs.transform.translation.x,tfs.transform.translation.y,0,0,0,tfs.transform.rotation.z,tfs.transform.rotation.w]
        except Exception as e:
            return [0,0,0,0,0,0,0]
    '''
        将四元数转换为角度输出
        输入的参数：四元数的zw 返回值：角度值
    '''
    def quaternion_to_Euler(self,qx, qy, qz, qw):

        R = [[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]]


        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], math.sqrt(R[2][1]**2 + R[2][2]**2))
        yaw = math.atan2(R[1][0], R[0][0])


        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return [roll, pitch, yaw]

    #播报语音
    #api.play_sound("/home/hgauto/catkin_ws/src/arv_navigation/voice/wxxw.wav")
    def play_sound(self,path):
        playsound(path)
        time.sleep(2)
        rospy.loginfo("播放音频成功")
        return True


    # 控制云台相机绕xz轴转动
    def pan_cam_control(self,data,data1):
        pub_data=rospy.Publisher('/Camera_Control_TOPIC',String,queue_size=100)
        x=data*3#角度分解
        i=0
        if data>0:
            while i<x:
                data_control='w'
                pub_data.publish(data_control)
                i+=1
                time.sleep(0.01)
                # print(i*5)
        elif data<0:
            x=-x
            i=0
            while i<x:
                data_control='s'
                pub_data.publish(data_control)
                i+=1
                time.sleep(0.01)
                # print(-i*5)
        y=data1*2
        i=0
        if data1>0:
            while i<y:
                data_control='a'
                pub_data.publish(data_control)
                i+=1
                time.sleep(0.01)
                # print(i)
        elif data1<0:
            y=-y
            i=0
            while i<y:
                data_control='d'
                pub_data.publish(data_control)
                i+=1
                time.sleep(0.01)
                # print(i)
        return True


    #云台相机图片设置
    #api.pan_cam_image_save("/home/hgauto/webviz/dist/sensor/2-2.jpeg","/camera/image")
    #需要启动相机输入保存路径和话题
    def pan_cam_image_save(self,path,topic_name):   
        cam0_path  = path 
        bridge = CvBridge()
        cv_img=rospy.wait_for_message(topic_name, Image)
        cv_img = bridge.imgmsg_to_cv2(cv_img, "bgr8")
        cv2.imwrite(cam0_path, cv_img)  #保存；
        return True

    # 获取darket识别结果 返回当前检测到的类
    # 启动darknet图像识别 放回类名称
    # data=api.get_class()
    # print(data)
    def get_curr_class(self):
        data_1=[]
        detect_data=rospy.wait_for_message("/darknet_ros/bounding_boxes",BoundingBoxes)
        for i in range(len(detect_data.bounding_boxes)) :
            data_1+=[[detect_data.bounding_boxes[i].probability,detect_data.bounding_boxes[i].Class,detect_data.bounding_boxes[i].xmin,detect_data.bounding_boxes[i].xmax,detect_data.bounding_boxes[i].ymin,detect_data.bounding_boxes[i].ymax] ]
        return data_1

    # 输出ocr识别结果并保存screen类的图像
    def class_crop(self,path,topic_name,class_name):
        cam0_path  = path 
        bridge = CvBridge()
        cv_img=rospy.wait_for_message(topic_name, Image)
        cv_img = bridge.imgmsg_to_cv2(cv_img, "bgr8")
        detect_data=rospy.wait_for_message("/darknet_ros/bounding_boxes",BoundingBoxes)
        if len(detect_data.bounding_boxes) >= 0:
            for i in range(len(detect_data.bounding_boxes)):
                if detect_data.bounding_boxes[i].Class ==class_name:
                    img_xmin = detect_data.bounding_boxes[i].xmin
                    img_xmax = detect_data.bounding_boxes[i].xmax
                    img_ymin = detect_data.bounding_boxes[i].ymin
                    img_ymax = detect_data.bounding_boxes[i].ymax
                    try:
                        get_img=cv_img[img_ymin:img_ymax,img_xmin:img_xmax]
                        cv2.imwrite(path, get_img)
                        rospy.loginfo("save image success ") 
                        return True
                    except:
                        return False



    # get=api.class_data("screen")
    # print(get)
    # 匹配输入的类
    # 匹配成功返回：True
    #匹配失败返回：None
    def class_data(self,name):
        detect_data=rospy.wait_for_message("/darknet_ros/bounding_boxes",BoundingBoxes)
        for i in range(len(detect_data.bounding_boxes)):
            if detect_data.bounding_boxes[i].Class ==name:
                return True
            else :
                return False

    def gas_dect(self):
        sensor_data=rospy.wait_for_message("/sensor_data",Float32MultiArray)
        # print(sensor_data)
        data={"酒精":sensor_data.data[1],"光强":sensor_data.data[9],"气体":sensor_data.data[0]}
    
        return data
        


    # LED灯控制
    # red_light_on='11'
    # red_light_off='21'
    # red_light_flashing='41'
    # yellow_light_on='12'
    # yellow_light_off='22'
    # yellow_light_flashing='42'
    # green_light_on='14'
    # green_light_off='24'
    # green_light_flashing='44'
    # buzzer_on='18'
    # buzzer_off='28'
    # buzzer_flashing='48'
    # auto.alarm_control("/dev/ttyUSB2",[21,14,18])
    def alarm_control(self,comPort,data):
        ser = serial.Serial(comPort, baudrate=9600, bytesize=8, parity='N', stopbits=1,timeout=None)
        if (ser.isOpen()):
            
            #ser.open()
            red_off='21'
            r=bytes.fromhex(red_off)
            ser.write(r)
            time.sleep(0.1)
            print("Redlight Init Done!!!")
            #初始化
            yellow_off='22'
            y=bytes.fromhex(yellow_off)
            ser.write(y)
            time.sleep(0.1)
            print("Yellowlight Init Done!!!")
            #初始化
            green_off='24'
            g=bytes.fromhex(green_off)
            ser.write(g)
            time.sleep(0.1)
            print("Greenlight Init Done!!!")
            #初始化
            buzzer_off='28'
            b=bytes.fromhex(buzzer_off)
            ser.write(b)
            time.sleep(0.1)
            print("Buzzer Init Done!!!")

            #输入控制数据
            for i in range(len(data)):
                a=str(data[i])
                d=bytes.fromhex(a)
                ser.write(d)
                time.sleep(0.1)
                print("Control Done!!!")
            return True
        else:
            print("无串口或串口被占用")
            return False

    # 开关控制
    def cabinet_off(self,cmd):
        if cmd ==1:
            data = '~/ws/src/Xiaomi-cloud-tokens-extractor/open.sh' 
            p = subprocess.Popen(data, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            line = p.stdout.readline()
            return True
        elif cmd ==0 :
            data = '~/ws/src/Xiaomi-cloud-tokens-extractor/close.sh' 
            p = subprocess.Popen(data, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            line = p.stdout.readline()
            return True

    def cabinet_ocr_read(self,name):
        path  = '~/ws/src/main/ocr.jpeg' 
        bridge = CvBridge()
        cv_img=rospy.wait_for_message("/camera/image", Image)
        cv_img = bridge.imgmsg_to_cv2(cv_img, "bgr8")
        detect_data=rospy.wait_for_message("/darknet_ros/bounding_boxes",BoundingBoxes)
        if len(detect_data.bounding_boxes) >= 0:
            for i in range(len(detect_data.bounding_boxes)):
                if detect_data.bounding_boxes[i].Class ==name:
                    img_xmin = detect_data.bounding_boxes[i].xmin
                    img_xmax = detect_data.bounding_boxes[i].xmax
                    img_ymin = detect_data.bounding_boxes[i].ymin
                    img_ymax = detect_data.bounding_boxes[i].ymax

                    get_img=cv_img[img_ymin:img_ymax,img_xmin:img_xmax]
                    cv2.imwrite(path, get_img)
                    reader= easyocr.Reader(['en'],gpu=False)
                    result = reader.readtext(path)
                    data=[]
                    for re in result:
                        data+=[re[1]]
                    end=time.time()
                    return data

    def mqtt_connect(self,host,port):
        # rospy.init_node("Truck_Mission_Node", anonymous=True)
        # self.host_ip = rospy.get_param("~host_ip", "192.168.3.3") # launch文件获取
        # self.host_port = rospy.get_param("~host_port", "50000")
        self.mission_nub = 0
        self.auto_req_msg = OrderedDict()
        self.auto_req_msg["name"] = "Auto"
        self.auto_req_msg["dir"] = "ZK"
        self.auto_req_msg["mission"] = 1
        self.auto_req_msg["state"] = "working"
        self.auto_req_json = json.dumps(self.auto_req_msg)
        self.ip = host
        self.port=port
        self.mqttClient = mqtt.Client()
        try:
            self.mqttClient.connect(self.ip, self.port, 60)
        except:
            return 0
        self.mqttClient.subscribe("/HG_DEV/ZK_ALL", 0)
        # self.mqttClient.subscribe("/HG_DEV/Auto_MOVE", 0)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数
        self.mqttClient.loop_start()
        return 1

    def on_publish(self, payload, topic="/HG_DEV/ZK_ALL_REQ", qos=2):
        # print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)
        time.sleep(2.0)

    def mqtt_send(self, RecdeviceName = 'ZK',msg =  'start'):
        self.auto_req_msg["dir"] = RecdeviceName
        self.auto_req_msg["msg"] = msg
        self.auto_req_json = json.dumps(self.auto_req_msg)
        self.on_publish(self.auto_req_json, "/HG_DEV/ZK_ALL", 2)
        return 1

    # mqtt回调函数
    def on_message_come(self, client, userdata, msg):
        self.sub_msg = json.loads(msg.payload.decode('utf-8'))
        if self.sub_msg["dir"]  == "Auto":
            self.mqtt_read_buf = str(self.sub_msg["msg"])


    def mqtt_read(self):
        self.mqtt_temp = self.mqtt_read_buf
        self.mqtt_read_buf = ""
        return self.mqtt_temp