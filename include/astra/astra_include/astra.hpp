#pragma once
#include "OpenNI.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "stdio.h"
#include <thread>
#include <unistd.h>
#include <functional>
#include <mutex>

using namespace cv;
using namespace std;
using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 1000
class astra
{
public:
    Mat data_3d_pts;
    Mat data_color;
    Mat data_depth;
    Mat data_normalize_depth;
    Mat data_fusion;
public:
    astra(int deviceID);
    bool astra_ctrl_one(int deviceID=0);
    bool astra_ctrl_two(int deviceID);
    void hMirrorTrans(const Mat &src, Mat &dst);
    bool open_astra();
    bool close_astra();
    bool is_opened();
    int get_dev_id();
    ~astra();
private:
    Status rc;
    Status rc_device;
    Status rc_depth;
    Status rc_color;
    Device device;
    VideoStream m_depthStream, m_colorStream;
    VideoMode color_mode,depth_mode;
    VideoFrameRef	m_depthFrame,m_colorFrame;
//    std::thread astra_thread;
    bool bDevOpen;
    bool continue_sample;
    bool is_waiting;
    int DevID;
    static mutex* mtx;
};
