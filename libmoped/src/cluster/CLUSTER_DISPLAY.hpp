#pragma once

namespace MopedNS {

	class CLUSTER_DISPLAY:public MopedAlg {

		int display;
	public:

		CLUSTER_DISPLAY( int display ) 
		: display(display) { }

		void getConfig( map<string,string> &config ) const {

			GET_CONFIG( display );
		}
			
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( display );
		}
		
		void process(FrameData &frameData) {
			
			if( display < 1 ) return;

			vector< vector< FrameData::Cluster > >&clusters = frameData.clusters;
			vector< vector< FrameData::Match > >   &matches = frameData.matches;
			
			int sz = 0;
			for(int model=0; model<(int)clusters.size(); model++ )
				sz += clusters[model].size();

			clog << _stepName << ":" << _alg << " : FOUND " << sz << " Clusters " << endl; 
		
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

				for(int model=0; model<(int)clusters.size(); model++ ) {
					
					int objectHash = 0; 
					for(unsigned int x=0; x<(*models)[model]->name.size(); x++) objectHash = objectHash ^ (*models)[model]->name[x];
					CvScalar color = objectColors[objectHash % 256];
					

					foreach( cluster, clusters[model] ) {
						
						if( cluster.size() &&  matches[model][ cluster.front() ].imageIdx == i ) {

							vector<Pt<2> > points; points.reserve( cluster.size() );
							foreach( match, cluster ) 
								points.push_back( matches[model][match].coord2D ); 
							
							list<Pt<2> > hull=getConvexHull(points); 
							eforeach( p, pt, hull ) {
								if( pt == hull.begin() ) continue;
								list<Pt<2> >::iterator ptm = pt; ptm--;
								cvLine(img, cvPoint( (*ptm)[0], (*ptm)[1]), cvPoint( p[0], p[1] ), color, 2, CV_AA );
							}
							cvLine(img, cvPoint( (*hull.begin())[0], (*hull.begin())[1]), cvPoint( (*hull.rbegin())[0], (*hull.rbegin())[1] ), color, 2, CV_AA );
						}
					}
				}
				
				cvShowImage( windowName.c_str(), img );
				
				cvReleaseImage(&img);
			}
			
			cvWaitKey( 1 );			
		}
	};
};
