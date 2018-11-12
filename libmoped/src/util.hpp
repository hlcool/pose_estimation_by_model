#pragma once

#include <omp.h>
#include <deque>

#include <opencv/cv.h>
#include <opencv/highgui.h>


#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 
#define eforeach( i, it, c ) for( typeof((c).begin()) it=(c).begin(), i##_hid = (c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && it!=(c).end(); (it==i##_hid)?++it,++i##_hid:i##_hid=it) for( typeof(*(c).begin()) &i=*it, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 

#define GET_CONFIG( varName ) config[ MopedNS::toString( _stepName ) + ":" + MopedNS::toString( _alg ) +":" + string(__FI##LE__).substr( ( string(__FI##LE__).find_last_of("/\\") + 1 + string(__FI##LE__).size() ) % string(__FI##LE__).size(), string(__FI##LE__).size() - ( string(__FI##LE__).find_last_of("/\\") + 1 + string(__FI##LE__).size() ) % string(__FI##LE__).size() -4  ) + "/" #varName ] = MopedNS::toString( varName )
#define SET_CONFIG( varName ) configUpdated = MopedNS::fromString( varName, config[ MopedNS::toString( _stepName ) + ":" + MopedNS::toString( _alg ) +":" + string(__FI##LE__).substr( ( string(__FI##LE__).find_last_of("/\\") + 1 + string(__FI##LE__).size() ) % string(__FI##LE__).size(), ( string(__FI##LE__).find_last_of("/\\") + 1 + string(__FI##LE__).size() ) % string(__FI##LE__).size() -4 ) +"/" #varName ] ) || configUpdated

namespace MopedNS {


struct FrameData {

    struct DetectedFeature {

        int imageIdx;

        // Image coordinate where the feature was detected
        Pt<2> coord2D;

        // Feature detected in the image
        vector<float> descriptor;
    };

    struct Match {

        int imageIdx;

        Pt<2> coord2D;

        Pt<3> coord3D;
    };

    typedef list<int> Cluster;

    vector<SP_Image> images;

    map< string, vector< DetectedFeature > > detectedFeatures;

    vector< vector< Match > > matches;

    vector< vector< Cluster> > clusters;

    list<SP_Object> *objects;





    int correctMatches, incorrectMatches;
    vector< vector< Cluster> > oldClusters;
    list<SP_Object> oldObjects;
    map<string, Float> times;
};


class MopedAlg {
public:


    vector<SP_Model> *models;

    bool capable;
    bool configUpdated;
public:

    string _stepName;
    int _alg;

    MopedAlg() { capable = true; configUpdated=true; }

    bool isCapable() const { return capable; }

    void setStepNameAndAlg( string &stepName, int alg ) {

        _stepName = stepName;
        _alg = alg;
    }

    virtual void modelsUpdated( vector<SP_Model> &_models) {

        models = &_models;
        configUpdated=true;
    }



    virtual void getConfig( map<string,string> &config ) const {}
    virtual void setConfig( map<string,string> &config ) {}

    virtual void process( FrameData &frameData ) = 0;
};


struct MopedStep : public vector< shared_ptr<MopedAlg> > {

    MopedAlg *getAlg() {
        foreach( alg, *this )
            if( alg->isCapable() )
                return alg.get();
        return NULL;
    }
};

struct MopedPipeline : public vector< MopedStep > {


    map< string, int > fromStepNameToIndex;


    void addAlg( string stepName, MopedAlg *mopedAlg ) {

        int step;
        if( fromStepNameToIndex.find( stepName ) == fromStepNameToIndex.end() ) {

            step = fromStepNameToIndex.size();
            fromStepNameToIndex[ stepName ] = step;
        } else {

            step = fromStepNameToIndex[ stepName ];
        }

        if( step >= (int)this->size() ) this->resize( step+1 );

        mopedAlg->setStepNameAndAlg( stepName, (*this)[step].size() );

        (*this)[step].push_back( shared_ptr<MopedAlg>( mopedAlg ) );
    }


    list<MopedAlg *> getAlgs( bool onlyActive = false ) {

        list<MopedAlg *> algs;

        foreach( mopedStep, *this )
            if( !onlyActive )
                foreach( alg, mopedStep )
                    algs.push_back( alg.get() );
            else
                if( mopedStep.getAlg() )
                    algs.push_back( mopedStep.getAlg() );

        return algs;
    }
};
}
