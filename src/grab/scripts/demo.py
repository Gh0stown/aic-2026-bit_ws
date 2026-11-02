#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from os import setgid
from turtle import pu
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult,MoveBaseActionGoal
from grab.srv  import    *
from std_srvs.srv  import    *
sortlist=[0,0,0,0,0,0]
a = -1
def callback(msg):
    global a
    global sortlist
    if msg.status.status == 3:  # 当前目标点状态为"REACHED"
        a=a+1
        rospy.loginfo('Goal reached')
        print (a)
        if(a==0):
            middle()  
        if(a==1): 
            sort0()  
            print ("前往第1个点")
        if(a==2):
            time.sleep(3)
            grab()
            put()
            time.sleep(6)
            sort1()
        if(a==3):
            time.sleep(3)
            grab()
            put()
            time.sleep(6)
            sort2()
        if(a==4):
            time.sleep(3)
            grab()
            put()
            time.sleep(6)
            sort3()
        if(a==5):
            time.sleep(3)
            grab()
            put()
            time.sleep(6)
            load()

def start():  #前往起始点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 0.0
    goal.goal.target_pose.pose.orientation.z = 0.0
    goal.goal.target_pose.pose.orientation.w = 0.0
      
    Npub.publish(goal)
def middle():  #前往中间点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 0.0
    goal.goal.target_pose.pose.orientation.z = 0.0
    goal.goal.target_pose.pose.orientation.w = -0.0
      
    Npub.publish(goal)

def sort0():  #前往1号点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 0.0
    goal.goal.target_pose.pose.orientation.z =  0.0
    goal.goal.target_pose.pose.orientation.w = -0.7
    Npub.publish(goal)
def sort1():  #前往2号点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 0.0
    goal.goal.target_pose.pose.orientation.z = 0.70
    goal.goal.target_pose.pose.orientation.w = -0.70
    Npub.publish(goal)
def sort2():  #前往3号点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 0.0
    goal.goal.target_pose.pose.orientation.z = 0.70
    goal.goal.target_pose.pose.orientation.w = -0.70
    Npub.publish(goal)
def sort3():  #前往4号点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 0.0
    goal.goal.target_pose.pose.position.y= 1.0
    goal.goal.target_pose.pose.orientation.z = 0.0
    goal.goal.target_pose.pose.orientation.w = -0.748426812095
    Npub.publish(goal)
def load():  #前往卸货点
    goal=MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id="map"
    goal.goal.target_pose.pose.position.x= 1.0
    goal.goal.target_pose.pose.position.y= 1.0
    goal.goal.target_pose.pose.orientation.z = 1
    goal.goal.target_pose.pose.orientation.w = 0
    Npub.publish(goal)







def put():   
    Grab("put")
    print("put")
def grab():   
    Grab("grab")
    print("grab")
def back():   
    Grab("back")
    print("back")
def ready():    
    Grab("ready")
    print("ready")
def unload():    
    Grab("unload")
    print("unload")


rospy.init_node('grab_navigation')
Npub = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=10)   #Topic:发送目标点
Nsub = rospy.Subscriber('/move_base/result',MoveBaseActionResult, callback)  #Topic:接受移动情况  3时为好
rospy.wait_for_service('grabserver')
Grab = rospy.ServiceProxy('grabserver',GrabServer)   #Service: 抓取服务 
time.sleep(1)

start()  #前往起始点
rospy.spin()


