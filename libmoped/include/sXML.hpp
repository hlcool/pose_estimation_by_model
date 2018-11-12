#pragma once

#include <vector>
#include <map>

#include <fstream>
#include <sstream>

class sXML {

    std::string getToken( std::istream &in ) {

        std::string s;
        while( !isspace(in.peek()) && in.peek()!='>' && in.peek()!='=' ) s+=in.get();
        while( isspace(in.peek()) ) in.get();
        return s;
    }

    void process( std::istream &in ) {

        children.clear();
        properties.clear();
        name.clear();

        while( in.get() != '<' );
        name = getToken( in );
        if( name=="" || name[0]=='/' || *name.rbegin()=='/' ) return;

        while( name == "!--" ) {
            int state = 0;
            while( state<2 || in.peek()!='>' )
                if( in.get() == '-' )
                    state++;
                else state=0;

            while( in.get() != '<' );
            name = getToken( in );
            if( name=="" || name[0]=='/' || *name.rbegin()=='/' ) return;
        }

        while( in.peek()!='/' ) {

            std::string propertyName = getToken( in );

            if( propertyName == "" || in.peek()!='=' ) break;

            std::string s;
            while( in.get()!='"' );
            while( in.peek()!='"' ) {
                if( in.peek()=='\\' ) {
                    in.get();
                    if( in.peek()=='n' ) { s+='\n';	in.get(); }
                }
                s+=in.get();
            } in.get();
            while( isspace(in.peek()) ) in.get();

            properties[propertyName] = s;
        }

        while( in.peek() == '>' ) {
            sXML s;
            s.process( in );
            if( s.name[0] == '/' ) return;

            if( s.name != "" )
                children.push_back(s);
        }

        while( in.peek() != '>' ) in.get();
    }

public:

    std::string name;
    std::vector<sXML> children;
    std::map<std::string, std::string> properties;

    sXML() {}

    bool fromFile( std::string &fileName ) { std::ifstream in( fileName.c_str(),  std::ifstream::in); return fromStream(in); }
    bool fromString( std::string &data ) { std::istringstream in(data); return fromStream(in); }
    bool fromStream( std::istream &in ) {

        std::ios_base::iostate originalExceptions = in.exceptions();
        try {

            in.exceptions ( std::ios_base::eofbit | std::ios_base::failbit | std::ios_base::badbit );
            process( in );
        } catch (std::ifstream::failure e) {

            in.exceptions( originalExceptions );
            return false;
        }
        in.exceptions( originalExceptions );
        return true;
    }

    void print(std::ostream &out, int level=0) const {

        std::string t; for(int x=0; x<level; x++) t+="\t";
        out << t << "<" << name;

        for( std::map<std::string, std::string>::const_iterator it=properties.begin(); it!=properties.end(); it++)
            out << " " << it->first << "=\"" << it->second << "\"";

        if( !children.empty() ) {

            out << ">" << std::endl;
            for( std::vector<sXML>::const_iterator it=children.begin(); it!=children.end(); it++)
                it->print(out, level+1);
            out << t << "</" << name << ">" << std::endl;

        } else {

            out << "/>" << std::endl;
        }
    }

    static std::string decode64( std::string data ) {

        static const char *base64Chars =
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "abcdefghijklmnopqrstuvwxyz"
                "0123456789+/";

        static int cD[4][256];
        for( int x=0; x<256; x++ )
            cD[0][x]=cD[1][x]=cD[2][x]=cD[3][x]=1<<24;

        for( int x=0; base64Chars[x]; x++ ) {
            cD[0][(unsigned int)base64Chars[x]]=x<<18;
            cD[1][(unsigned int)base64Chars[x]]=x<<12;
            cD[2][(unsigned int)base64Chars[x]]=x<<6;
            cD[3][(unsigned int)base64Chars[x]]=x<<0;
        }

        std::string ret; ret.reserve( 3*sizeof(data)/4 );
        unsigned int d=0, i=0;
        for(unsigned int x=0; x<data.size(); x++) {
            if( cD[0][(int)data[x]] == 1<<24 ) continue;
            d = d | cD[i][(int)data[x]];
            if( ++i==4 ) {
                ret+=(char)((d>>16)&255);
                ret+=(char)((d>>8)&255);
                ret+=(char)((d>>0)&255);
                d=i=0;
            }
        }
        ret+=(char)((d>>16)&255);
        ret+=(char)((d>>8)&255);
        ret+=(char)((d>>0)&255);

        ret.resize( ret.size() - (i?4-i:3) );
        return ret;
    }

    friend std::ostream& operator<< (std::ostream &out, const sXML &s) { s.print( out ); return out; }
    std::string &operator[] ( std::string n ) { return properties[n]; }
    std::string &operator[] ( const char *n) { return properties[std::string(n)]; }
};
