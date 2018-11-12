#pragma once

//#define DEBUG
#define SIFT_OPENGL
//#define SHOW_IMG
//#define SHOW_TIME

//#define NORMAL_RES
//#define LOW_RES
#define HIGH_RES

#include <util/UTIL_UNDISTORT.hpp>

#ifdef SIFT_OPENGL
#include <feat/FEAT_SIFT_GPU.hpp>
#else
#include <feat/FEAT_SIFT_CPU.hpp>
#endif

#include <match/MATCH_ANN_CPU.hpp>
#include <cluster/CLUSTER_MEAN_SHIFT_CPU.hpp>
#include <pose/POSE_RANSAC_LM_DIFF_REPROJECTION_CPU.hpp>
#include <filter/FILTER_PROJECTION_CPU.hpp>

#ifdef DEBUG
#include <feat/FEAT_DISPLAY.hpp>
#include <match/MATCH_DISPLAY.hpp>
#include <cluster/CLUSTER_DISPLAY.hpp>
#include <pose/POSE_DISPLAY.hpp>
#endif

#ifdef SHOW_IMG
#include <GLOBAL_DISPLAY.hpp>
#endif

#ifdef SHOW_TIME
#include <STATUS_DISPLAY.hpp>
#endif

namespace MopedNS 
{
void createPipeline( MopedPipeline &pipeline )
{
    pipeline.addAlg( "UNDISTORTED_IMAGE", new UTIL_UNDISTORT );


    /************************************************************************/
#ifdef NORMAL_RES

#ifdef SIFT_OPENGL
    pipeline.addAlg( "SIFT", new FEAT_SIFT_GPU("-1", "0", ":0.0", "-1") );
#else
    pipeline.addAlg( "SIFT", new FEAT_SIFT_CPU("-1") );
#endif

#ifdef DEBUG
    pipeline.addAlg( "FEAT_DISPLAY", new FEAT_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "MATCH_SIFT", new MATCH_ANN_CPU( 128, "SIFT", 5., 0.8) );
#ifdef DEBUG
    pipeline.addAlg( "MATCH_DISPLAY", new MATCH_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "CLUSTER", new CLUSTER_MEAN_SHIFT_CPU( 200, 20, 7, 100) );
#ifdef DEBUG
    pipeline.addAlg( "CLUSTER_DISPLAY", new CLUSTER_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "POSE", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 600, 200, 4, 5, 6, 10) );
#ifdef DEBUG
    pipeline.addAlg( "POSE_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "FILTER", new FILTER_PROJECTION_CPU( 5, 4096., 2) );
    pipeline.addAlg( "POSE2", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 100, 500, 4, 6, 8, 5) );
    pipeline.addAlg( "FILTER2", new FILTER_PROJECTION_CPU( 7, 4096., 3) );
#ifdef DEBUG
    pipeline.addAlg( "POSE2_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

#endif


    /************************************************************************/
#ifdef LOW_RES

#ifdef SIFT_OPENGL
    pipeline.addAlg( "SIFT", new FEAT_SIFT_GPU("-1", "0", ":0.0", "-1") );
#else
    pipeline.addAlg( "SIFT", new FEAT_SIFT_CPU("-1") );
#endif

#ifdef DEBUG
    pipeline.addAlg( "FEAT_DISPLAY", new FEAT_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "MATCH_SIFT", new MATCH_ANN_CPU( 128, "SIFT", 5., 0.75) );
#ifdef DEBUG    cv::drawContours(frame, hulls, -1, cv::Scalar(255, 255, 255), -1);
    cv::imshow("hull", frame);
    cv::waitKey(10);
    pipeline.addAlg( "MATCH_DISPLAY", new MATCH_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "CLUSTER", new CLUSTER_MEAN_SHIFT_CPU( 60, 20, 7, 100) );
#ifdef DEBUG
    pipeline.addAlg( "CLUSTER_DISPLAY", new CLUSTER_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "POSE", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 300, 200, 4, 5, 6, 10) );
#ifdef DEBUG
    pipeline.addAlg( "POSE_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "FILTER", new FILTER_PROJECTION_CPU( 6, 4096., 2) );
    pipeline.addAlg( "POSE2", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 30, 500, 4, 6, 8, 5) );
    pipeline.addAlg( "FILTER2", new FILTER_PROJECTION_CPU( 8, 4096., 3) );
#ifdef DEBUG
    pipeline.addAlg( "POSE2_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

#endif


    /************************************************************************/
#ifdef HIGH_RES

#ifdef SIFT_OPENGL
    pipeline.addAlg( "SIFT", new FEAT_SIFT_GPU("-1", "0", ":0.0", "-1") );
#else
    pipeline.addAlg( "SIFT", new FEAT_SIFT_CPU("-1") );
#endif

#ifdef DEBUG
    pipeline.addAlg( "FEAT_DISPLAY", new FEAT_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "MATCH_SIFT", new MATCH_ANN_CPU( 128, "SIFT", 5., 0.75) );
#ifdef DEBUG
    pipeline.addAlg( "MATCH_DISPLAY", new MATCH_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "CLUSTER", new CLUSTER_MEAN_SHIFT_CPU( 150, 20, 7, 100) );
#ifdef DEBUG
    pipeline.addAlg( "CLUSTER_DISPLAY", new CLUSTER_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "POSE", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 50, 200, 4, 5, 6, 10) );
#ifdef DEBUG
    pipeline.addAlg( "POSE_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

    pipeline.addAlg( "FILTER", new FILTER_PROJECTION_CPU( 6, 4096., 2) );
    pipeline.addAlg( "POSE2", new POSE_RANSAC_LM_DIFF_REPROJECTION_CPU( 50, 500, 4, 6, 8, 5) );
    pipeline.addAlg( "FILTER2", new FILTER_PROJECTION_CPU( 8, 4096., 3) );
#ifdef DEBUG
    pipeline.addAlg( "POSE2_DISPLAY", new POSE_DISPLAY( 2 ) );
#endif

#endif



#ifdef SHOW_IMG
    pipeline.addAlg( "GLOBAL_DISPLAY", new GLOBAL_DISPLAY( 2 ) );
#endif

#ifdef SHOW_TIME
    pipeline.addAlg( "STATUS_DISPLAY", new STATUS_DISPLAY( 1 ) );
#endif
}
}
