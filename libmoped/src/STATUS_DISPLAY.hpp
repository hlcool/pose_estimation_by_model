#pragma once

namespace MopedNS {

	class STATUS_DISPLAY:public MopedAlg {

		int display;
	public:

		STATUS_DISPLAY( int display ) 
		: display(display) { }

		void getConfig( map<string,string> &config ) const {

			GET_CONFIG( display );
		}
			
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( display );
		}
		
		void process(FrameData &frameData) {
			
			if( !display ) return;
			
			foreach( times, frameData.times ) {
				cout << times.first << ": " << times.second*1000 << "ms" <<endl;
			}
		}
	};
};
