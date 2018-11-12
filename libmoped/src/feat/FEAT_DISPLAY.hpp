#pragma once

namespace MopedNS {

	class FEAT_DISPLAY:public MopedAlg {

		int display;
	public:

		FEAT_DISPLAY( int display ) 
		: display(display) { }

		void getConfig( map<string,string> &config ) const {

			GET_CONFIG( display );
		}
			
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( display );
		}
		
		void process(FrameData &frameData) {
			
			if( display < 1 ) return;
			
			
			clog << "FEATURES DETECTED: ";
			foreach( df, frameData.detectedFeatures ) 
				clog << df.second.size() << " " << df.first << " ";
			clog << endl;
			
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
				
				foreach( featureKind, frameData.detectedFeatures ) {
					
					int objectHash = 0; 
					for(unsigned int x=0; x<featureKind.first.size(); x++) objectHash = objectHash ^ featureKind.first[x];
					CvScalar color = objectColors[objectHash % 256];
					
					foreach( corresp, featureKind.second ) {
						if( corresp.imageIdx==i )
							cvCircle(img, cvPoint( corresp.coord2D[0], corresp.coord2D[1]), 2, color , CV_FILLED, CV_AA );
					}
				}
				
				cvShowImage( windowName.c_str(), img );
				
				cvReleaseImage(&img);
			}			
			
			cvWaitKey( 1 );			
		}
	};
};
