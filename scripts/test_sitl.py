#!/usr/bin/env python
# coding=utf-8
import rospy
#添加command 消息
from px4_command.msg import command

#设置模式
Idle=0 #默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
Takeoff=1
Move_ENU=2
Move_Body=3
Hold=4
Land=5
Disarm=6
Failsafe_land=7
#定义command对象
command_now=command()


def talker():
    comid = 0
    sleep_time=10
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('/px4/command', command, queue_size=10)
    rospy.init_node('takeoff_test', anonymous=True)
    rate = rospy.Rate(1)
    #更新频率是1hz
    a=input("please input 1 for takeoff:")
    if(a==1):
        print("takeoff")
        command_now.command = Takeoff
        pub.publish(command_now)
        print("complete takeoff")
        rate.sleep()

    a = input("please input 2 for move:")
    if(a==2):
        i=0
        print("climb")
        while(i<sleep_time):
        #>>>>>>>>>>>>>>>>>>>>>>>>爬升到5m>>>>>>>>>>>>>>>>>>>>>
            command_now.command = Move_Body;
            command_now.sub_mode = 0;#位置控制模式
            command_now.pos_sp[0] = 0;
            command_now.pos_sp[1] = 0;
            command_now.pos_sp[2] = 5;
            command_now.yaw_sp = 0;
            command_now.comid = comid;
            comid=comid+1;
            pub.publish(command_now)
            print("complete climb")
            rate.sleep()
            i=i+1


        print("complete climb")

    while not rospy.is_shutdown():
        print("move")
        command_now.command = Move_Body;     #//机体系下移动
        command_now.comid = comid;
        comid=comid+1;
        command_now.sub_mode = 2; #xy 速度控制模式 z 位置控制模式
        command_now.vel_sp[0] =  2;
        command_now.vel_sp[1] =  0;
        command_now.pos_sp[2] =  0;
        command_now.yaw_sp = 0;
        rate.sleep()
        print("complete move")
        pub.publish(command_now)
if __name__ == '__main__':

    talker()
