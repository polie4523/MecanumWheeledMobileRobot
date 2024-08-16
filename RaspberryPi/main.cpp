#include <iostream>
#include "MecanumWMR.h"

int main(int argc, char** argv) {
    MecanumWMR wmr;
    if (wmr.init()==false) return 1;
    wmr.setInitCond(0,0,0);
    wmr.setPIDgain_rotation(1,0.25,0.25);
    
    std::array<float, 3> target_point = {0.0};
    for (int i=0;i<2;i++) {
        std::cout<<"Please enter a target point [x, y, theta]: ";
        std::cin>>target_point[0]>>target_point[1]>>target_point[2];
        wmr.Point2PointMove(target_point);
        auto p_est = wmr.getPosition();
        std::cout<<"current point [x, y, theta]: "<<p_est[0]<<","<<p_est[1]<<","<<p_est[2]<<std::endl;
    }
    wmr.shutdown();

	return 0;
}


