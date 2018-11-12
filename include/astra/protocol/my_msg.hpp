#pragma once
#include <map>
#include <string>
//#include <unordered_map>
using namespace std;

namespace my_msg
{
    enum my_msg{photo=1,climb_stairs,rgbcamera_calibration,kickball,open_vision,close_vision,include_angle_calibration};
    map<string,my_msg> str2my_msg;
//    std::unordered_map<string,my_msg> str2my_msg{{"photo",photo},{"climb_stairs",climb_stairs},{"calibration",calibration},{"kickball",kickball},{"open_vision",open_vision},{"close_vision",close_vision}};
    
    void protocol_initialize()
    {
        str2my_msg["photo"]=photo;
        str2my_msg["climb_stairs"]=climb_stairs;
        str2my_msg["rgbcamera_calibration"]=rgbcamera_calibration;
        str2my_msg["include_angle_calibration"]=include_angle_calibration;
        str2my_msg["kickball"]=kickball;
        str2my_msg["open_vision"]=open_vision;
        str2my_msg["close_vision"]=close_vision;
    }
}
