#pragma once

#include <ANN.h>

namespace MopedNS {

class MATCH_ANN_CPU:public MopedAlg {

    static inline void norm( vector<float> &d ) {
        float norm=0; for (int x=0; x<(int)d.size(); x++) norm += d[x]*d[x]; norm = 1./sqrtf(norm);
        for (int x=0; x<(int)d.size(); x++) d[x] *=norm;
    }

    int DescriptorSize;
    string DescriptorType;
    Float Quality;
    Float Ratio;

    bool skipCalculation;

    vector<int> correspModel;
    vector< Pt<3> * > correspFeat;

    ANNkd_tree *kdtree;


    void Update() {

        skipCalculation = true;

        unsigned int modelsNFeats = 0;
        foreach( model, *models )
            modelsNFeats += model->IPs[DescriptorType].size();

        correspModel.resize( modelsNFeats );
        correspFeat.resize( modelsNFeats );

        ANNpointArray refPts = annAllocPts( modelsNFeats , DescriptorSize );

        int x=0;
        for( int nModel = 0; nModel < (int)models->size(); nModel++ ) {

            vector<Model::IP> &IPs = (*models)[nModel]->IPs[DescriptorType];

            for( int nFeat = 0; nFeat < (int)IPs.size(); nFeat++ ) {

                correspModel[x] = nModel;
                correspFeat[x]  = &IPs[nFeat].coord3D;
                norm( IPs[nFeat].descriptor );
                for( int i=0; i<DescriptorSize; i++ )
                    refPts[x][i] = IPs[nFeat].descriptor[i];

                x++;
            }
        }

        if( modelsNFeats > 1 ) {

            skipCalculation = false;
            if( kdtree ) delete kdtree;
            kdtree = new ANNkd_tree(refPts, modelsNFeats, DescriptorSize );
        }
        configUpdated = false;
    }

public:

    MATCH_ANN_CPU( int DescriptorSize, string DescriptorType, Float Quality, Float Ratio )
        : DescriptorSize(DescriptorSize), DescriptorType(DescriptorType), Quality(Quality), Ratio(Ratio)  {

        kdtree=NULL;
        skipCalculation=true;
    }

    void getConfig( map<string,string> &config ) const {

        GET_CONFIG(DescriptorType);
        GET_CONFIG(DescriptorSize);
        GET_CONFIG(Quality);
        GET_CONFIG(Ratio);
    };

    void setConfig( map<string,string> &config ) {

        SET_CONFIG(DescriptorType);
        SET_CONFIG(DescriptorSize);
        SET_CONFIG(Quality);
        SET_CONFIG(Ratio);
    };

    void process( FrameData &frameData ) {

        if( configUpdated ) Update();

        if( skipCalculation ) return;

        vector< FrameData::DetectedFeature > &corresp = frameData.detectedFeatures[DescriptorType];
        if( corresp.empty() ) return;

        vector< vector< FrameData::Match > > &matches = frameData.matches;
        matches.resize( models->size() );



        ANNpoint pt = annAllocPt(DescriptorSize);

        ANNidxArray	nx = new ANNidx[2];
        ANNdistArray ds = new ANNdist[2];

        for( int i=0; i<(int)corresp.size(); i++)  {

            norm( corresp[i].descriptor);
            for (int j = 0; j < DescriptorSize; j++)
                pt[j] = corresp[i].descriptor[j];

#pragma omp critical(ANN)
            kdtree->annkSearch(pt, 2, nx, ds, Quality);


            if(  ds[0]/ds[1] < Ratio ) {

                int nModel1 = correspModel[nx[0]];
                if( matches[nModel1].capacity() < 1000 ) matches[nModel1].reserve(1000);
                matches[nModel1].resize( matches[nModel1].size() +1 );

                FrameData::Match &match = matches[nModel1].back();

                match.imageIdx = corresp[i].imageIdx;
                match.coord3D = *correspFeat[nx[0]];
                match.coord2D = corresp[i].coord2D;
            }
        }
    }
};
};
