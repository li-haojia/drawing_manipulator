#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/String.h>
#include<pthread.h>
#include<iostream>
#include<stdio.h>

using namespace std;
ros::Publisher pub;
geometry_msgs::Twist vel_cmd;
pthread_t pth_[5];

void* vel_ctr(void* arg)
{
    while(true)
    {

        pub.publish(vel_cmd);
        ros::spinOnce();
        sleep(1);
    }
    return 0;
}

void callback(const std_msgs::String::ConstPtr& msg)
{
    cout<<"好的："<<msg->data.c_str()<<endl;
    string str1 = msg->data.c_str();
    string str2 = "向前。";
    string str3 = "后退。";
    string str4 = "左转。";
    string str5 = "右转。";
    string str6 = "停止。";
    if(str1 == str2)
    {
        cout<<"11111"<<endl;
        vel_cmd.linear.x = 0.5;
        vel_cmd.angular.z = 0;
        pthread_create(&pth_[0],NULL,vel_ctr,NULL);
    }
    if(str1 == str3)
    {
        vel_cmd.linear.x = -0.5;
        vel_cmd.angular.z = 0;
        pthread_create(&pth_[1],NULL,vel_ctr,NULL);
    }
    if(str1 == str4)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = 0.4;
        pthread_create(&pth_[2],NULL,vel_ctr,NULL);
    }
    if(str1 == str5)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = -0.4;
        pthread_create(&pth_[3],NULL,vel_ctr,NULL);
    }
    if(str1 == str6)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = 0;
        pthread_create(&pth_[0],NULL,vel_ctr,NULL);
    }    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_word");
    ros::NodeHandle n;
    
    pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    ros::Subscriber sub = n.subscribe("voiceWords",10,callback);
    cout<<"您好！可以语音控制啦！"<<endl;
    cout<<"向前行———————————>向前"<<endl;
    cout<<"向后退———————————>后退"<<endl;
    cout<<"向左转———————————>左转"<<endl;
    cout<<"向右转———————————>右转"<<endl;
    cout<<"使停止———————————>停止"<<endl;
    ros::spin();
}
