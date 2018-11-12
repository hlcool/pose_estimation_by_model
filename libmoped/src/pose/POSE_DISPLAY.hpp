#pragma once
#include <lm.h>

namespace MopedNS {

	class POSE_DISPLAY:public MopedAlg {

		int display;
	public:

		POSE_DISPLAY( int display ) 
		: display(display) { }

		void getConfig( map<string,string> &config ) const {

			GET_CONFIG( display );
		}
			
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( display );
		}
		
		void process(FrameData &frameData) {
			
			if( display < 1 ) return;
			
			clog << _stepName << ":" << _alg << " : FOUND " << frameData.objects->size() << " Objects" << endl; 
			
			if( display < 2 ) return;
				
			vector<CvScalar> objectColors(256);
			for( int i=0; i<256; i++ )  {
				
				int r = i;
				r = (((r * 214013L + 2531011L) >> 16) & 32767);
				int a = 192 + r%64;
				r = (((r * 214013L + 2531011L) >> 16) & 32767);
				int b = 64 + r%128;
				r = (((r * 214013L + 2531011L) >> 16) & 32767);
				int c = r%64;
				for(int x=0; x<100; x++) if( (r = (((r * 214013L + 2531011L) >> 16) & 32767)) % 2 ) swap(a,b); else swap(a,c);
				
				objectColors[i] = cvScalar( a,b,c );
			}
			
			for( int i=0; i<(int)frameData.images.size(); i++) {
				
				string windowName = _stepName + " #" + toString(i) + ":" + frameData.images[i]->name;
				
				cvNamedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE);
				
				IplImage* img = cvCreateImage(cvSize(frameData.images[i]->width,frameData.images[i]->height), IPL_DEPTH_8U, 3);

				for (int y = 0; y < frameData.images[i]->height; y++) {
					for (int x = 0; x < frameData.images[i]->width; x++) { 
						img->imageData[y*img->widthStep+3*x + 0] = frameData.images[i]->data[y*frameData.images[i]->width + x];
						img->imageData[y*img->widthStep+3*x + 1] = frameData.images[i]->data[y*frameData.images[i]->width + x];
						img->imageData[y*img->widthStep+3*x + 2] = frameData.images[i]->data[y*frameData.images[i]->width + x];
					}
				}

				foreach( object, *frameData.objects ) {
					
					int objectHash = 0;
					for(unsigned int x=0; x<object->model->name.size(); x++) objectHash = objectHash ^ object->model->name[x];
					CvScalar color = objectColors[objectHash % 256];

				
					bool skip = false;
					Pt<2> bBox2D[2][2][2];
					for(int x=0; x<2; x++) {
						for(int y=0; y<2; y++) {
							for(int z=0; z<2; z++) {
								Pt<3> bBox3D;
								bBox3D.init( object->model->boundingBox[x][0], object->model->boundingBox[y][1], object->model->boundingBox[z][2] );
								bBox2D[x][y][z] = project( object->pose, bBox3D, *frameData.images[i].get() );
								skip = skip || bBox2D[x][y][z][0]==0 || bBox2D[x][y][z][1]==0;
							}
						}
					}
					
					if( skip ) continue;
					
					for(int x1=0; x1<2; x1++) 
						for(int y1=0; y1<2; y1++) 
							for(int z1=0; z1<2; z1++) 
								for(int x2=0; x2<2; x2++) 
									for(int y2=0; y2<2; y2++) 
										for(int z2=0; z2<2; z2++) 
											if( ((x1==x2)?1:0) + ((y1==y2)?1:0) + ((z1==z2)?1:0) == 2 )
												cvLine(img, cvPoint( bBox2D[x1][y1][z1][0], bBox2D[x1][y1][z1][1]), cvPoint( bBox2D[x2][y2][z2][0], bBox2D[x2][y2][z2][1] ), color, 2, CV_AA);
				}
				
				cvShowImage( windowName.c_str(), img );
				
				cvReleaseImage(&img);
			}

			cvWaitKey( 1 );			
		}
	};
};
