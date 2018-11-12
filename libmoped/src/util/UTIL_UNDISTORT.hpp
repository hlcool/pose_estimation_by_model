#pragma once

namespace MopedNS {

class UTIL_UNDISTORT :public MopedAlg {

    struct CompareCameraParameters {
        bool operator() (const SP_Image& i1, const SP_Image& i2) const {

            if( i1->width != i2->width )
                return i1->width<i2->width;
            else if( i1->height != i2->height )
                return i1->height<i2->height;
            else if( i1->intrinsicLinearCalibration != i2->intrinsicLinearCalibration )
                return i1->intrinsicLinearCalibration < i2->intrinsicLinearCalibration;
            else
                return i1->intrinsicNonlinearCalibration < i2->intrinsicNonlinearCalibration;
        }
    };

    map< SP_Image, pair<IplImage*,IplImage*>, CompareCameraParameters > distortionMaps;

    void init( const SP_Image& i ) {

        CvSize imageSize = cvSize( i->width, i->height );
        IplImage *MapX = cvCreateImage( imageSize, IPL_DEPTH_32F, 1);
        IplImage *MapY = cvCreateImage( imageSize, IPL_DEPTH_32F, 1);

        float kk[9]={0};
        for(int x=0; x<9; x++) kk[x]=0;

        kk[0] = i->intrinsicLinearCalibration[0];
        kk[2] =	i->intrinsicLinearCalibration[2];
        kk[4] =	i->intrinsicLinearCalibration[1];
        kk[5] =	i->intrinsicLinearCalibration[3];
        kk[8] = 1.;
        CvMat cvK = cvMat(3, 3, CV_32FC1, kk);

        //zjt
        float kk_c[8];
        for(int x=0; x<8; x++) kk_c[x]= i->intrinsicNonlinearCalibration[x];
        CvMat dist = cvMat( 8, 1, CV_32FC1, kk_c );


#ifdef HAVE_CV_UNDISTORT_RECTIFY_MAP
        float feye[9] = {1,0,0,0,1,0,0,0,1};
        CvMat eye = cvMat(3,3,CV_32F, feye);
        cvInitUndistortRectifyMap(&cvK, &dist, NULL, &cvK, MapX, MapY);
#else
        cvInitUndistortMap(&cvK, &dist, MapX, MapY);
#endif

        distortionMaps[ i ] = make_pair( MapX, MapY );
    }

public:


    void process( FrameData &frameData ) {

#pragma omp parallel for
        for( int i=0; i<(int)frameData.images.size(); i++) {

            Image *image = frameData.images[i].get();
            pair<IplImage*,IplImage*> *maps;

#pragma omp critical(UNDISTORT)
            {
                maps = &distortionMaps[ frameData.images[i] ];

                if( !maps->first || !maps->second ) {
                    init( frameData.images[i] );
                    maps = &distortionMaps[ frameData.images[i] ];
                }
            }

            IplImage* gs = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);

            for (int y = 0; y < image->height; y++)
                memcpy( &gs->imageData[y*gs->widthStep], &image->data[y*image->width], image->width );

            IplImage *img = cvCloneImage(gs);
            cvRemap( img, gs, maps->first, maps->second, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
            cvReleaseImage(&img);

            for (int y = 0; y < image->height; y++)
                memcpy( &image->data[y*image->width], &gs->imageData[y*gs->widthStep], image->width );

            cvReleaseImage(&gs);
        }
    }
};
};
