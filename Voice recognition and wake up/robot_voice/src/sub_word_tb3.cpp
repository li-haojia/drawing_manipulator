#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<std_msgs/String.h>
#include<pthread.h>
#include<iostream>
#include<stdio.h>
#include <fstream>

using namespace std;
ros::Publisher pub;
geometry_msgs::Pose vel_cmd;
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
bool exists_test (const std::string name) {
    ifstream f(name);
    return f.good();
}

void callback(const std_msgs::String::ConstPtr& msg)
{
    if(!exists_test("/home/cjx/snowboy/examples/recognition.txt"))
    {
        return;
    }

    cout<<"好的："<<msg->data.c_str()<<endl;
    string str1 = msg->data.c_str();
    string str2 = "出发";
    if(str1.find(str2)!= string::npos )
    {
        cout<<"11111"<<endl;
        pub.publish(vel_cmd);
        remove("/home/cjx/snowboy/examples/recognition.txt");
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_word_tb3");
    ros::NodeHandle n;
    
    pub = n.advertise<geometry_msgs::Pose>("/now_start",1);
    ros::Subscriber sub = n.subscribe("voiceWords",10,callback);
    cout<<"您好！可以语音控制啦！"<<endl;
    cout<<"出发———————————>出发"<<endl;
    ros::spin();
}
