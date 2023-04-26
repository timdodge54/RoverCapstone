#include <ros/ros.h>
#include "SDL2/SDL.h"
#include <motor_controls/Int16List.h>

#include <iostream>
#include <vector>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "sender_node");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<motor_controls::Int16List>("chatter", 1);
    ros::Rate loop_rate(30);
    
    int sdlResult = SDL_Init(SDL_INIT_GAMECONTROLLER);
    SDL_GameController *mController;
    mController = SDL_GameControllerOpen(0);

    if (!mController) 
    {
        ROS_INFO("NO CONTROLLER");
    }

    float scale = 1.0f;
    float prevRightTrig = 0;
    float prevLeftTrig = 0;
    float prevStickX = 0;
    float rightTrig;
    float leftTrig;
    float rightStickX;

    int loopsFailed = 0;


    const float TRIGGER_MAX = 32767.0f;

    while (ros::ok())
    {
        SDL_Event event;

        while(SDL_PollEvent(&event)) {
            switch (event.type) {

            case SDL_QUIT:
                break;
            }
        }

        motor_controls::Int16List msg;
        std::vector<short int> stuff (2, 0);
        msg.data = stuff;


        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) >= 6000)
        {
            rightTrig = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        }
        else
        {
            rightTrig = 0;
        }

        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERLEFT) >= 6000)
        {
            leftTrig = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        }
        else
        {   
            leftTrig = 0;
        }

        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) >= 6000 || SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) <= -6000)
        {
            rightStickX = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX);
        }
        else
        {
            rightStickX = 0;
        }

	msg.data[0] = static_cast<int>((rightTrig - leftTrig) / TRIGGER_MAX * 100);
	msg.data[1] = static_cast<int>(rightStickX / TRIGGER_MAX *100);

        ROS_INFO("[%d, %d]", msg.data[0], msg.data[1]);

	if(rightTrig != prevRightTrig ||  leftTrig != prevLeftTrig || rightStickX != prevStickX)
	{
		chatter_pub.publish(msg);
		prevRightTrig = rightTrig;
		prevLeftTrig = leftTrig;
		prevStickX = rightStickX;
		ros::spinOnce();
	}
	else
	{
		loopsFailed++;
		if(loopsFailed > 15)
		{
			chatter_pub.publish(msg);
			
		}
	}
        

        loop_rate.sleep();
    }
    ROS_INFO("IM NOT OKAY");
    return 0;
}
