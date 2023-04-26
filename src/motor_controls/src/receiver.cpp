#include <ros/ros.h>
#include <motor_controls/Int16List.h>
#include <stdio.h>
#include "serialib.h"
#include <string>
//#include "std_msgs/Int8MultiArray.h"


serialib serial;


void sendState(int state[]){
	std::string msg="<"+std::to_string(state[0])+","+std::to_string(state[1])+">";
	for (int i=0;i<msg.length();i++){  
	serial.writeChar(msg[i]);
	printf("%c",msg[i]);
	}
	return;
}

void chatterCallback(const motor_controls::Int16List::ConstPtr& msg)
{

//    ROS_INFO("th:%d z:%d r:%d sig:%d alpha:%d gamma:%d]", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);    
	
	int state[2];
	state[0] = msg->data[0];
	state[1] = msg->data[1];
	sendState(state);

}


int main(int argc, char **argv)
{	
    char errorOpening = serial.openDevice("/dev/ttyACM0", 9600);
    printf("hello\n"); 
    printf("%c",errorOpening);
	
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;

}
