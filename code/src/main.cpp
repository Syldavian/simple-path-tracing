#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"

#include <string>

using namespace std;

//based on smallpt
Vector3f radiance(SceneParser* parser, Ray &r, int depth, unsigned short *Xi){
    //Do intersection
    Hit hit;
    Group* baseGroup=parser->getGroup();
    if(!baseGroup->intersect(r,hit,1e-3))
        return parser->getBackgroundColor(); //if miss, return background color

    //if hit
    Vector3f rd=r.getDirection();
    float t=hit.getT(); //distance to intersection
    Vector3f n=hit.getNormal().normalized(); //hit object normal
    Vector3f x=r.pointAtParameter(t); //intersection point
    //properly oriented normal:
    //When a ray hits a glass surface, the ray tracer must determine 
    //if it is entering or exiting glass to compute the refraction ray.
    //The dot product of the normal and ray direction tells this
    Vector3f nl;
    if(Vector3f::dot(n,rd)<0)
        nl=n;
    else
        nl=-n;
    Material *m=hit.getMaterial();
    Vector3f f=m->getDiffuseColor(); //get object color
    if(hit.color.x()>=-1e-3)
        f=hit.color;
    Vector3f ecolor=m->getSpecularColor();//get emission color
    if(depth>30) 
        return ecolor;//max depth

    //Stop the recursion randomly based on the surface reflectivity.
    //Use the maximum component (r,g,b) of the surface color.
    float p=f.x()>max(f.y(),f.z())?f.x():max(f.y(),f.z());
    if(++depth>5){
        if(erand48(Xi)<p)
            f=f*(1/p);
        else
            return ecolor;
    }
    
    int reflection=m->getReflectionType();

    if(reflection==0){
        //For diffuse (not shiny) reflection
        //construct random ray
        float r1=2*M_PI*erand48(Xi);//get random angle
        float r2=erand48(Xi);
        float r2s=sqrt(r2);//get random distance from center 
        //Use normal to create orthonormal coordinate frame (w,u,v)
        Vector3f w=nl;
        Vector3f u=Vector3f::cross((fabs(w.x())>0.1?Vector3f(0,1,0):Vector3f(1,0,0)), w).normalized();
        Vector3f v=Vector3f::cross(w,u);
        Vector3f d=(u*cos(r1)*r2s+v*sin(r1)*r2s+w*sqrt(1-r2)).normalized();//random reflect ray
        Ray randomReflRay(x,d);
        return ecolor+f*radiance(parser,randomReflRay,depth,Xi);
    } 
    else if(reflection==1){// Ideal SPECULAR reflection
        Ray reflRay(x,rd-n*2*Vector3f::dot(n,rd));
        return ecolor+f*radiance(parser,reflRay,depth,Xi);
    } 
    else{// Ideal dielectric REFRACTION
        //Glass is both reflective and refractive, so we compute the reflected ray here.
        Ray reflRay(x,rd-n*2*Vector3f::dot(n,rd));
        bool into=Vector3f::dot(n,nl)>0;//Determine if ray is entering or exiting glass
        float nc=1, nt=1.5, nnt=into?nc/nt:nt/nc, ddn=Vector3f::dot(rd,nl), cos2t;
        if((cos2t=1-nnt*nnt*(1-ddn*ddn))<0)//Total internal reflection
            return ecolor+f*radiance(parser,reflRay,depth,Xi);
        Vector3f tdir=(rd*nnt-n*((into?1:-1)*(ddn*nnt+sqrt(cos2t)))).normalized();//Compute the refracted ray
        float a=nt-nc, b=nt+nc, R0=a*a/(b*b), c=1-(into?-ddn:Vector3f::dot(tdir,n));
        float Re=R0+(1-R0)*c*c*c*c*c, Tr=1-Re, P=0.25+0.5*Re, RP=Re/P, TP=Tr/(1-P);
        Ray refractRay(x,tdir);
        return ecolor+f*(depth>2 ? (erand48(Xi)<P ? // Russian roulette,Reflect or Refract using Fresnel Term
            radiance(parser,reflRay,depth,Xi)*RP:radiance(parser,refractRay,depth,Xi)*TP) :
            radiance(parser,reflRay,depth,Xi)*Re+radiance(parser,refractRay,depth,Xi)*Tr);
    }
}

float clamp(float x) { return x<0 ? 0 : x>1 ? 1 : x; }
float gamma(float x) { return pow(clamp(x),1/2.2); }//applies a gamma correction of 2.2

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    SceneParser sceneParser(inputFile.c_str());
    Camera* camera=sceneParser.getCamera();
    int width=camera->getWidth();
    int height=camera->getHeight();
    Image img(width,height);
    int samps=1000;
    cout<<"Group size:"<<sceneParser.getGroup()->getGroupSize()<<endl;

#pragma omp parallel for schedule(dynamic, 1) 
    for(int y=0;y<height;y++){
        fprintf(stderr,"\rRendering (%d spp) %5.2f%%",samps,100.*y/(height-1));
        for(unsigned short x=0, Xi[3]={0,0,y*y*y}; x<width; x++){
            Vector3f finalColor=Vector3f::ZERO;
            //SSAA
            for(int sy=0;sy<2;sy++){
                for(int sx=0;sx<2;sx++){
                    //soft shadow
                    for(int s=0;s<samps;s++){
                        //Tent Filter
                        double r1=2*erand48(Xi), dx=r1<1 ? sqrt(r1)-1 : 1-sqrt(2-r1);
                        double r2=2*erand48(Xi), dy=r1<1 ? sqrt(r2)-1 : 1-sqrt(2-r2);
                        Ray r(camera->generateRay(Vector2f((sx+0.5+dx)/2+x,(sy+0.5+dy)/2+y), Xi));
                        finalColor+=radiance(&sceneParser,r,0,Xi)*(0.25/samps);
                    }
                }
            }            
            finalColor=Vector3f(clamp(finalColor[0]),clamp(finalColor[1]),clamp(finalColor[2]));
            finalColor=Vector3f(gamma(finalColor[0]),gamma(finalColor[1]),gamma(finalColor[2]));
            img.SetPixel(x,y,finalColor);
        }
    }

    cout << "Hello! Computer Graphics!" << endl;
    img.SaveBMP(outputFile.c_str());
    return 0;
}

