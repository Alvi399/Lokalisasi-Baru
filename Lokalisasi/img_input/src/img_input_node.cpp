#include "img_input/img_input.h"

int main(int argc,char **argv){

    ros::init(argc,argv,"img_input_node");

    BallDetector ball_detector;

    ros::Rate loop_rate(30);

    while(ros::ok()){

        ball_detector.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
