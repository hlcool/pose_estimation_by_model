#pragma once

namespace MopedNS {

class FILTER_PROJECTION_CPU :public MopedAlg {

    int MinPoints;
    Float FeatureDistance;
    Float MinScore;

public:
    // MinScore is optional
    FILTER_PROJECTION_CPU( int MinPoints, Float FeatureDistance )
        : MinPoints(MinPoints), FeatureDistance(FeatureDistance), MinScore(0) {
    }

    FILTER_PROJECTION_CPU( int MinPoints, Float FeatureDistance, Float MinScore )
        : MinPoints(MinPoints), FeatureDistance(FeatureDistance), MinScore(MinScore) {
    }

    void getConfig( map<string,string> &config ) const {

        GET_CONFIG( MinPoints );
        GET_CONFIG( FeatureDistance );
        GET_CONFIG( MinScore );
    }

    void setConfig( map<string,string> &config ) {

        SET_CONFIG( MinPoints );
        SET_CONFIG( FeatureDistance );
        SET_CONFIG( MinScore );
    }

    void process( FrameData &frameData ) {

        vector< SP_Image > &images = frameData.images;
        vector< vector< FrameData::Match > > &matches = frameData.matches;

        // Sanity check (problems when GPU is not behaving)
        if (matches.size() < models->size())
            return;

        map< pair<Pt<2>, Image *>, pair< Float, Object *> > bestPoints;

        //map< Object *, int > matchesPerObject;

        map< Object *, FrameData::Cluster > newClusters;

        for(int m=0; m<(int)models->size(); m++) {
            foreach( object, *frameData.objects) {
                if( object->model->name == (*models)[m]->name ) {


                    FrameData::Cluster &newCluster = newClusters[ object.get() ];
                    Float score = 0;
                    for(int match=0; match<(int)matches[m].size(); match++ ) {

                        Pt<2> p = project( object->pose, matches[m][match].coord3D, *images[matches[m][match].imageIdx] );
                        p -= matches[m][match].coord2D;
                        float projectionError = p[0]*p[0]+p[1]*p[1];
                        if( projectionError < FeatureDistance ) {
                            newCluster.push_back( match );
                            score+=1./(projectionError + 1.);

                        }
                    }

                    object->score = score;

                    // Go through the list of matches, and transfer each potential match to the object with max score
                    foreach( match, newCluster ) {

                        pair< Float, Object *> &point = bestPoints[make_pair(matches[m][match].coord2D, images[matches[m][match].imageIdx].get())];
                        if( point.first < score ) {

                            //matchesPerObject[point.second]=max(0, matchesPerObject[point.second]-1);

                            point.first = score;
                            point.second = object.get();

                            //matchesPerObject[point.second]++;
                        }
                    }
                }
            }
        }

        // Now put only the best points in each cluster
        newClusters.clear();
        for(int m=0; m<(int)models->size(); m++) {
            for (int match = 0; match < (int) matches[m].size(); match++) {
                // Find out the object that this point belongs to, and put it in the object's cluster
                pair< Float, Object *> &point = bestPoints[make_pair(matches[m][match].coord2D, images[matches[m][match].imageIdx].get())];
                if ( point.second != NULL && point.second->model->name == (*models)[m]->name )
                    newClusters[ point.second ].push_back( match );
            }
        }

        frameData.clusters.clear();
        frameData.clusters.resize( models->size() );

        for(int m=0; m<(int)models->size(); m++) {
            eforeach( object, object_it, *frameData.objects) {
                if( object->model->name == (*models)[m]->name ) {
                    // Delete object instance if not reliable
                    if( (int)newClusters[object.get()].size() < MinPoints || object->score < MinScore) {
                        //if( matchesPerObject[object.get()] < MinPoints || object->score < MinScore) {
                        object_it = frameData.objects->erase(object_it);
                    } else {
                        frameData.clusters[m].push_back( newClusters[object.get()] );
                    }
                }
            }
        }

    }
};
};
