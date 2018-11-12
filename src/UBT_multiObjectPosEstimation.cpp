#include <UBT_multiObjectPosEstimation.hpp>
#include <moped.hpp>
#include <dirent.h>
#include <iostream>

using namespace MopedNS;

#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2)
#define MARKER_SIZE 30

Moped *moped;
vector<string> modelNames;

double fx_ubt, fy_ubt, cx_ubt, cy_ubt;
double k1_ubt, k2_ubt, k3_ubt, k4_ubt, k5_ubt, k6_ubt, p1_ubt, p2_ubt;

bool is_track = false;
cv::Mat obj_ROI;
vector< vector<cv::Point> > pre_objects_hull;

vector<Point3f> worldPoints;
vector<Point2f> markerCoords;
Mat camera_matrix;
Mat distortion_coefficients;


static void quat2rot(cv::Mat Rt, cv::Mat &rvec, cv::Mat &tvec)
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


static string decodeRect(Mat src, pair<string, Mat> pose)
{
    cv::Mat rvec, tvec;
    quat2rot(pose.second, rvec, tvec);
    vector<Point2f> imagePoints, onePoints;
    vector<int> decodeMarker;
    projectPoints(worldPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

    for(int i=0; i<12; i++)
    {
        onePoints.push_back(imagePoints[i]);
        if((i+1)%4==0)
        {
            Mat marker_image;
            Mat M = getPerspectiveTransform(onePoints, markerCoords);
            warpPerspective(src, marker_image, M, Size(MARKER_SIZE, MARKER_SIZE));
            threshold(marker_image, marker_image, 115, 255, THRESH_BINARY);
            int none_zero_count = countNonZero(marker_image);
            if (none_zero_count > marker_image.rows*marker_image.cols/6)
            {
                decodeMarker.push_back(1);
            }
            else
            {
                decodeMarker.push_back(0);
            }
            onePoints.clear();
        }
    }

    int markerID = decodeMarker[0]*4+decodeMarker[1]*2+decodeMarker[2];
    string label = format("%s", format("%d", markerID).c_str());
    return label;
}

bool initPoseDetector(string models_path, string camera_info)
{
    moped = new Moped;
    DIR *dp;
    struct dirent *dirp;

    if((dp  = opendir(models_path.c_str())) ==  nullptr)
    {
        std::cout<< "the models_path is error..." << std::endl;
        return false;
    }

    vector<string> fileNames;
    while((dirp = readdir(dp)) != nullptr)
    {
        string fileName =  models_path + "/" + string(dirp->d_name);
        if( fileName.rfind(".moped.xml") != string::npos )
        {
            fileNames.push_back( fileName );
            continue;
        }
    }

    if(fileNames.size()<=0)
    {
        std::cout<< "there is no 3D object model...." << std::endl;
        return false;
    }

    for(int i=0; i<(int)fileNames.size(); ++i)
    {
        sXML XMLModel;
        XMLModel.fromFile(fileNames[i]);

        string Mname = moped->addModel(XMLModel);
        modelNames.push_back(Mname);
    }
    closedir(dp);

    if(camera_info.empty())
    {
        std::cout<< "the camera_info is error..." << std::endl;
        return false;
    }
    FileStorage file_storage(camera_info, FileStorage::READ);
    if(!file_storage.isOpened())
    {
        cout << "load camrea matrix error..." << endl;
        return false;
    }
    file_storage["camera_matrix"] >> camera_matrix;
    file_storage["distortion_coefficients"] >> distortion_coefficients;
    file_storage.release();

    fx_ubt = camera_matrix.at<double>(0,0);
    fy_ubt = camera_matrix.at<double>(1,1);
    cx_ubt = camera_matrix.at<double>(0,2);
    cy_ubt = camera_matrix.at<double>(1,2);


    k1_ubt = distortion_coefficients.at<double>(0,0);
    k2_ubt = distortion_coefficients.at<double>(1,0);
    p1_ubt = distortion_coefficients.at<double>(2,0);
    p2_ubt = distortion_coefficients.at<double>(3,0);
    k3_ubt = distortion_coefficients.at<double>(4,0);
    k4_ubt = 0.;
    k5_ubt = 0.;
    k6_ubt = 0.;
    if(distortion_coefficients.rows == 8)
    {
        k4_ubt = distortion_coefficients.at<double>(5,0);
        k5_ubt = distortion_coefficients.at<double>(6,0);
        k6_ubt = distortion_coefficients.at<double>(7,0);
    }

#ifdef USE_RECT

    worldPoints.push_back(Point3d(10, -10, 0));
    worldPoints.push_back(Point3d(10, -6, 0));
    worldPoints.push_back(Point3d(14, -6, 0));
    worldPoints.push_back(Point3d(14, -10, 0));

    worldPoints.push_back(Point3d(29, -10, 0));
    worldPoints.push_back(Point3d(29, -6, 0));
    worldPoints.push_back(Point3d(33, -6, 0));
    worldPoints.push_back(Point3d(33, -10, 0));

    worldPoints.push_back(Point3d(48, -10, 0));
    worldPoints.push_back(Point3d(48, -6, 0));
    worldPoints.push_back(Point3d(52, -6, 0));
    worldPoints.push_back(Point3d(52, -10, 0));

    markerCoords.push_back(Point2f(0, 0));
    markerCoords.push_back(Point2f(0, MARKER_SIZE-1));
    markerCoords.push_back(Point2f(MARKER_SIZE-1, MARKER_SIZE-1));
    markerCoords.push_back(Point2f(MARKER_SIZE-1, 0));

#endif

    return true;
}

static bool detectPose(Mat frame, vector< pair<string, Mat> > &result, vector< vector<cv::Point> > &objects_hull)
{
    vector<SP_Image> images;
    SP_Image mopedImage(new Image);

    mopedImage->intrinsicLinearCalibration.init(fx_ubt, fy_ubt, cx_ubt, cy_ubt);
    mopedImage->intrinsicNonlinearCalibration.init(k1_ubt, k2_ubt, p1_ubt, p2_ubt, k3_ubt, k4_ubt, k5_ubt, k6_ubt);
    mopedImage->cameraPose.translation.init(0., 0., 0.);
    mopedImage->cameraPose.rotation.init(0., 0., 0., 1.);

    mopedImage->data.resize(frame.cols * frame.rows);
    mopedImage->data = (vector<unsigned char>)(frame.reshape(1,1));
    mopedImage->width = frame.cols;
    mopedImage->height = frame.rows;
    mopedImage->name = "frame";
    images.push_back(mopedImage);
    list<SP_Object> objects;
    moped->processImages(images, objects);

    result.clear();
    objects_hull.clear();
    if(objects.size()>0)
    {
        vector< vector<SP_Object> > UBT_OBJ(modelNames.size());
        list<SP_Object>::iterator iter_l;
        for(iter_l=objects.begin(); iter_l!=objects.end(); ++iter_l)
        {
            SP_Object temp = *iter_l;
            if(temp->score <10)
                continue;

            for(size_t t=0; t<modelNames.size(); ++t)
            {
                if(temp->model->name == modelNames[t])
                {
                    UBT_OBJ[t].push_back(temp);
                    break;
                }
            }
        }


        for(size_t k=0; k<modelNames.size(); ++k)
        {
            if(UBT_OBJ[k].size()>0)
            {
                float max_score=0.;
                int idx=-1;
                for(size_t i=0; i<UBT_OBJ[k].size(); i++)
                {
                    if(UBT_OBJ[k][i]->score > max_score)
                    {
                        max_score = UBT_OBJ[k][i]->score;
                        idx = i;
                    }
                }

                list<Pt<2> > hull = UBT_OBJ[k][idx]->getObjectHull((Image &)*(images[0].get()));
                vector<cv::Point> each_hull;
                foreach( pt, hull)
                {
                    cv::Point hull_point = cv::Point((int) pt[0], (int) pt[1]);
                    each_hull.push_back(hull_point);
                }
                objects_hull.push_back(each_hull);


                UBT_OBJ[k][idx]->pose.rotation.norm();
                float flip = UBT_OBJ[k][idx]->pose.rotation[0] + UBT_OBJ[k][idx]->pose.rotation[1] + UBT_OBJ[k][idx]->pose.rotation[2] + UBT_OBJ[k][idx]->pose.rotation[3];
                if(flip < 0)
                {
                    UBT_OBJ[k][idx]->pose.rotation[0] *= -1;
                    UBT_OBJ[k][idx]->pose.rotation[1] *= -1;
                    UBT_OBJ[k][idx]->pose.rotation[2] *= -1;
                    UBT_OBJ[k][idx]->pose.rotation[3] *= -1;
                }

                Mat RT = (Mat_<double>(1,7)<<UBT_OBJ[k][idx]->pose.rotation[0],
                        UBT_OBJ[k][idx]->pose.rotation[1],
                        UBT_OBJ[k][idx]->pose.rotation[2],
                        UBT_OBJ[k][idx]->pose.rotation[3],
                        UBT_OBJ[k][idx]->pose.translation[0]*1000.0,
                        UBT_OBJ[k][idx]->pose.translation[1]*1000.0,
                        UBT_OBJ[k][idx]->pose.translation[2]*1000.0);
                result.push_back(make_pair(UBT_OBJ[k][idx]->model->name, RT));
            }
        }
    }

    if(result.size()>0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

vector< pair<string, Mat> > applyPoseDetector(Mat frame)
{
    vector< pair<string, Mat> > objects_pose;
    vector< vector<cv::Point> > cur_objects_hull;
    objects_pose.clear();
    cur_objects_hull.clear();

    cvtColor(frame, frame, COLOR_BGR2GRAY);
    if(!is_track)
    {
        bool  is_detect = detectPose(frame, objects_pose, cur_objects_hull);
        if(is_detect)
        {
            is_track = true;
            pre_objects_hull.clear();
            pre_objects_hull.swap(cur_objects_hull);
            objects_pose.clear();
        }
    }
    else
    {
        cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());
        cv::drawContours(mask, pre_objects_hull, -1, cv::Scalar(255), -1);
        cv::bitwise_and(frame, mask, obj_ROI);
        bool  is_detect = detectPose(obj_ROI, objects_pose, cur_objects_hull);
        if(is_detect)
        {
            for(size_t t=0; t<objects_pose.size(); ++t)
            {
                if(objects_pose[t].first == "8")
                {
                    continue;
                }


#ifdef USE_RECT

                string label = decodeRect(frame, objects_pose[t]);
                objects_pose[t].first = label;

#endif
            }

            is_track = true;
            pre_objects_hull.clear();
            pre_objects_hull.swap(cur_objects_hull);
        }
        else
        {
            objects_pose.clear();
            is_track = false;
        }
    }

    return objects_pose;
}

void destroyPoseDetector()
{
    if(moped != nullptr)
    {
        delete moped;
    }
    moped = nullptr;
}
