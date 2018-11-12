#include <string.h>
#include <moped.hpp>
#include <util.hpp>

#include <config.hpp>

#include <cstdio>
#include <iostream>

using namespace MopedNS;
using namespace std;

struct Moped::MopedPimpl {

    MopedPipeline pipeline;
    vector<SP_Model> models;


    MopedPimpl() {

        createPipeline( pipeline );

        map<string,string> config = getConfig();
        setConfig( config );

        modelsUpdated();
    }


    map<string,string> getConfig() {

        map<string,string> config;

        list<MopedAlg *> algs = pipeline.getAlgs();
        foreach( alg, algs )
            alg->getConfig( config );

        return config;
    }


    void setConfig( map<string,string> &config) {

        list<MopedAlg *> algs=pipeline.getAlgs();
        foreach( alg, algs )
            alg->setConfig( config );
    }


    void modelsUpdated() {

        list<MopedAlg *> algs=pipeline.getAlgs( true );
        foreach( alg, algs )
            alg->modelsUpdated( models );
    }

    string addModel( sXML &sxml ) {

        SP_Model m(new Model);

        m->name = sxml["name"];

        m->boundingBox[0].init( 10E10, 10E10, 10E10 );
        m->boundingBox[1].init(-10E10,-10E10,-10E10 );

        sXML *points = NULL;
        foreach( pts, sxml.children)
            if( pts.name == "Points" )
                points = &pts;

        if( points == NULL ) return "";

        foreach( pt, points->children ) {

            Model::IP ip;

            std::istringstream iss( pt["p3d"] );
            iss >> ip.coord3D;

            m->boundingBox[0].min( ip.coord3D );
            m->boundingBox[1].max( ip.coord3D );

            std::istringstream jss( pt["desc"] );
            Float f;
            while( jss >> f ) ip.descriptor.push_back(f);

            m->IPs[ pt["desc_type"] ].push_back(ip);
        }

        addModel( m );

        return m->name;
    }

    void addModel( SP_Model &model ) {

        int found = false;
        foreach( m, models )
            if( (found = (m->name == model->name ) ) )
                m = model;

        if( !found )
            models.push_back(model);

        modelsUpdated();
    }

    void removeModel( const string &name ) {

        eforeach( model, model_it, models )
                if( model->name == name )
                model_it = models.erase(model_it);

        modelsUpdated();
    }

    const vector<SP_Model> &getModels() const  {

        return models;
    }

    int processImages( vector<SP_Image> &images, list<SP_Object> &objects ) {

        foreach( image, images )
            image->TM.init( image->cameraPose );

        objects.clear();


        FrameData frameData;


        frameData.objects = &objects;
        frameData.images = images;

        list<MopedAlg *> algs=pipeline.getAlgs( true );


        struct timespec tStep, tEnd;
        foreach( alg, algs ) {

            clock_gettime(CLOCK_REALTIME, &tStep);
            alg->process( frameData );
            clock_gettime(CLOCK_REALTIME, &tEnd);
            Float tstep = ( (tEnd.tv_sec -  tStep.tv_sec)*1000000000LL + tEnd.tv_nsec -  tStep.tv_nsec )/1000000000.;
            //cout << "-----" << tstep*1000 << endl;
            frameData.times[alg->_stepName] = tstep;
        }

        return objects.size();
    }

    vector<shared_ptr<sXML> > createPlanarModelsFromImages( vector<MopedNS::SP_Image> &images, Float scale ) {

        foreach( image, images )
            image->TM.init( image->cameraPose );

        list<SP_Object> objects;

        FrameData frameData;
        frameData.objects = &objects;
        frameData.images = images;

        list<MopedAlg *> algs=pipeline.getAlgs( true );
        foreach( alg, algs )
            alg->process( frameData );

        vector<shared_ptr<sXML> > xmls;
        foreach( image, images ) {
            xmls.push_back( shared_ptr<sXML>(new sXML) );
            xmls.back()->name="Model";
            (*xmls.back())["name"]=image->name;
            xmls.back()->children.resize(1);
            xmls.back()->children[0].name="Points";
        }

        foreach( vd, frameData.detectedFeatures ) {
            foreach( d, vd.second) {

                sXML point;
                point.name = "Point";
                point["p3d"] = toString(d.coord2D[0]*scale)+" "+toString(d.coord2D[1]*scale)+" 0";
                point["desc_type"] = vd.first;

                for(int x=0; x<(int)d.descriptor.size(); x++)
                    point["desc"] += string(x?" ":"") + toString(d.descriptor[x]);

                xmls[d.imageIdx]->children[0].children.push_back(point);
            }
        }

        return xmls;
    }

};

Moped::Moped() { mopedPimpl = new MopedPimpl(); }
Moped::~Moped() { delete mopedPimpl; }

map<string,string> Moped::getConfig() { 
    return mopedPimpl->getConfig(); }

void Moped::setConfig( map<string,string> &config) { 
    mopedPimpl->setConfig( config ); }

string Moped::addModel( sXML &sxml ) {
    return mopedPimpl->addModel( sxml ); }

void Moped::addModel( SP_Model &model ) { 
    mopedPimpl->addModel( model ); }

void Moped::removeModel( const string &name ) { 
    mopedPimpl->removeModel( name ); }

const vector<SP_Model> &Moped::getModels() const { 
    return mopedPimpl->getModels(); }

int Moped::processImages( vector<SP_Image> &images, list<SP_Object> &objects ) { 
    return mopedPimpl->processImages( images, objects ); }

vector<std::tr1::shared_ptr<sXML> > Moped::createPlanarModelsFromImages( vector<MopedNS::SP_Image> &images, float scale ) {
    return mopedPimpl->createPlanarModelsFromImages( images, scale ); }

