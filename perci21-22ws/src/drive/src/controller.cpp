#include <ros/ros.h>
#include "SDL2/SDL.h"
#include <drive/int8List.h>
#include <drive/boolList.h>

#include <iostream>
#include <vector>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "piDrive");
    ros::NodeHandle nh;
    ros::Publisher drive_pub = nh.advertise<drive::int8List>("cDrive", 1);
    ros::Publisher camera_pub = nh.advertise<drive::boolList>("cCamera", 1);
    ros::Rate loop_rate(30);
    
    int sdlResult = SDL_Init(SDL_INIT_GAMECONTROLLER);
    SDL_GameController *mController;
    mController = SDL_GameControllerOpen(0);

    if (!mController) 
    {
        ROS_INFO("NO CONTROLLER");
    }

    float scale = .4f;
    float prevRightTrig = 0;
    float prevLeftTrig = 0;
    float prevStickX = 0;
    float rightTrig;
    float leftTrig;
    float rightStickX;
    float scaleFactor = 0.4f;
    float scaleFactorTurn = .35f;

    bool cycleForward = 0;
    bool prevForward = cycleForward;
    bool cycleBack = 0;
    bool prevBack = cycleBack;
    bool quadMode = 0;
    bool prevQuad = quadMode;

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

        drive::int8List msg;
        std::vector<int8_t> stuff (6, -1);
        msg.data = stuff;


        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) >= 6000)
        {
            rightTrig = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) - 6000;
        }
        else
        {
            rightTrig = 0;
        }

        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERLEFT) >= 6000)
        {
            leftTrig = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_TRIGGERLEFT) - 6000;
        }
        else
        {   
            leftTrig = 0;
        }

        if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) >= 6000 || SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) <= -6000)
        {
		if(SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) > 0)
		{rightStickX = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) - 6000;
        	}
		else
		{
		rightStickX = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX) + 6000;
 
		}
	}
        else
        {
            rightStickX = 0;
        }

	msg.data[0] = static_cast<int>((rightTrig - leftTrig) / TRIGGER_MAX * 100 * scaleFactor);
	msg.data[1] = static_cast<int>(rightStickX / TRIGGER_MAX *100 * scaleFactorTurn);

        ROS_INFO("[%d, %d]", msg.data[0], msg.data[1]);

	if(rightTrig != prevRightTrig ||  leftTrig != prevLeftTrig || rightStickX != prevStickX)
	{
		drive_pub.publish(msg);
		prevRightTrig = rightTrig;
		prevLeftTrig = leftTrig;
		prevStickX = rightStickX;
		ros::spinOnce();
		loopsFailed = 0;
	}
	else
	{
		loopsFailed++;
		if(loopsFailed >= 15)
		{
			drive_pub.publish(msg);
			loopsFailed = 0;
		}
	}
       
	if(SDL_GameControllerGetButton(mController, SDL_CONTROLLER_BUTTON_DPAD_RIGHT) > 0)
	{
		cycleForward = 1;
		cycleBack = 0;
		quadMode = 0;
	}
	else if( SDL_GameControllerGetButton(mController, SDL_CONTROLLER_BUTTON_DPAD_LEFT) > 0)
	{
		cycleForward = 0;
		cycleBack = 1;
		quadMode = 0;

	}
	else if(std::max(SDL_GameControllerGetButton(mController, SDL_CONTROLLER_BUTTON_DPAD_UP),  SDL_GameControllerGetButton(mController, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) > 0)
	{
		cycleForward = 0;
		cycleBack = 0;
		quadMode = 1;

	}
	else
	{
		cycleForward = 0;
		cycleBack = 0;
		quadMode = 0;

	}	

	if(cycleForward != prevForward ||  cycleBack != prevBack || quadMode != prevQuad)
	{
		std::vector<uint8_t> stuff(3, 0);
		drive::boolList cam;
		cam.data = stuff;
		cam.data[0] = cycleForward;
		cam.data[1] = cycleBack;
		cam.data[2] = quadMode;
		camera_pub.publish(cam);
		prevForward = cycleForward;
		prevBack = cycleBack;
		prevQuad = quadMode;
		ros::spinOnce();
	}		

        loop_rate.sleep();
    }
    ROS_INFO("IM NOT OKAY");
    return 0;
}
