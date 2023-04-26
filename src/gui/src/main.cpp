#include <ros/ros.h>
#include "SDL2/SDL.h"
#include <gui/int8List.h>

#include <iostream>
#include <vector>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "piGui");
    ros::NodeHandle nh;
    ros::Publisher drive_pub = nh.advertise<gui::int8List>("cGui", 1);
    ros::Rate loop_rate(30);

    SDL_Window m_window = SDL_CreateWindow("Perci", 0, 0, 910, 930, 0);
    SDL_Renderer m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    IMG_Init(IMG_INIT_PNG);

    int sdlResult = SDL_Init(SDL_INIT_GAMECONTROLLER);
    SDL_GameController *mController;
    mController = SDL_GameControllerOpen(0);

    if (!mController)
    {
        ROS_INFO("NO CONTROLLER");
    }



    while(true)
    {
	
    }
    return 0;
}
