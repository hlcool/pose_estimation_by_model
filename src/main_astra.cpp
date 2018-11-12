#include "astra.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "UBT_multiObjectPosEstimation.hpp"
#include <math.h>
#include <omp.h>

using namespace cv;
using namespace std;

void quat2rot(cv::Mat Rt, cv::Mat &rvec, cv::Mat &tvec)
{
    double x, y, z, w, x2, y2, z2, w2, xy, xz, yz, wx, wy, wz;

    x = Rt.at<double>(0,0);
    y = Rt.at<double>(0,1);
    z = Rt.at<double>(0,2);
    w = Rt.at<double>(0,3);
    
    x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w;
    xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z;
    wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z;

    cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
    R.at<double>(0,0) = w2+x2-y2-z2; R.at<double>(0,1) = xy-wz; R.at<double>(0,2) = xz+wy;
    R.at<double>(1,0) = xy+wz; R.at<double>(1,1) = w2-x2+y2-z2; R.at<double>(1,2) = yz-wx;
    R.at<double>(2,0) = xz-wy; R.at<double>(2,1) = yz+wx; R.at<double>(2,2) = w2-x2-y2+z2;
    cv::Rodrigues(R, rvec);

    tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    tvec.at<double>(0,0) = Rt.at<double>(0,4);
    tvec.at<double>(1,0) = Rt.at<double>(0,5);
    tvec.at<double>(2,0) = Rt.at<double>(0,6);
}

void myDrawAxis(cv::Mat &frame, cv::Mat camera_matrix, cv::Mat distortion_coefficients, cv::Mat rvec, cv::Mat tvec, float length)
{
    vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

    cv::line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 2);
}

int main(int argc, char **argv)
{
    omp_set_num_threads(4);
    initPoseDetector(argv[1], argv[2]);
    
    Mat camera_matrix = Mat(3, 3, CV_64FC1);
    Mat distortion_coefficients = Mat(5, 1, CV_64FC1);
    FileStorage file_storage(argv[2], FileStorage::READ);
    if(!file_storage.isOpened())
    {
        cout << "load camrea matrix error..." << endl;
        return -1;
    }
    file_storage["camera_matrix"] >> camera_matrix;
    file_storage["distortion_coefficients"] >> distortion_coefficients;
    file_storage.release();

    astra Astra(1);
    Mat src;
    while(1)
    {
        waitKey(1);
        if(Astra.data_color.empty())
            continue;

        flip(Astra.data_color,src,1);
        double t1 = (double)cv::getTickCount();
        vector< pair<string, Mat> > result = applyPoseDetector(src);
        double t2 = (double)cv::getTickCount();
        cout<<"total time:"<<(t2-t1)*1000/(cv::getTickFrequency())<<endl;

        if(result.size()>0)
        {
            for(size_t i=0; i<result.size(); i++)
            {
                cv::Mat rvec, tvec;
                quat2rot(result[i].second, rvec, tvec);
                myDrawAxis(src, camera_matrix, distortion_coefficients, rvec, tvec, 50);
                putText(src, result[i].first, Point(0, 40), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 255, 0), 2);
                cout << "ID:" <<result[i].first <<"  "<<result[i].second << endl;
            }
        }

        imshow("video", src);
        cout << "-------------------------pose----------------------------" << endl;

        if(cv::waitKey(1)>0)
            break;
    }
    destroyPoseDetector();

    return 1;
}
