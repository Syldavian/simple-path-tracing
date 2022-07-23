#ifndef PLANE_H
#define PLANE_H

#include "image.hpp"
#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {
        normal=Vector3f::UP;
        d=0;
        image=nullptr;
    }

    Plane(const Vector3f &normal, float d, Material *m, Image *img=nullptr, bool tex=false) : 
            Object3D(m),d(d),material(m),image(img),useTexture(tex){
        this->normal=normal.normalized();
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f o=r.getOrigin();
        Vector3f dir=r.getDirection();
        dir.normalize();
        float cos=Vector3f::dot(normal,dir);
        if(fabs(cos)<1e-4) 
            return false;
        float t=(d-Vector3f::dot(normal,o))/cos;
        if(t<tmin||t>h.getT()) 
            return false;
        Vector3f intersection=o+dir*t;
        if(useTexture){
            float u_, v_;
            if(fabs(normal.x())>=1-1e-3&&fabs(normal.x())<=1+1e-3){
                u_=intersection.z();
                v_=intersection.y();
            }
            else if(fabs(normal.y())>=1-1e-3&&fabs(normal.y())<=1+1e-3){
                u_=intersection.x();
                v_=intersection.z();
            }
            else if(fabs(normal.z())>=1-1e-3&&fabs(normal.z())<=1+1e-3){
                u_=intersection.x();
                v_=intersection.y();
            }
            else{
                h.set(t,material,normal);
                return true;
            }
            int u=abs(fmod(u_,image->Width()));
            int v=abs(fmod(v_,image->Height()));
            int compressed_width = image->Width()/5;
            int compressed_height = image->Height()/5;
            u=u%compressed_width;
            v=v%compressed_height;
            int cx=5*u+2, cy =5*v+2;
            Vector3f final_color(0,0,0);
            for(int i=cx-2;i<=cx+2;i++){
                for(int j=cy-2;j<=cy+2;j++)
                    final_color=final_color+image->GetPixel(i,j);
            }
            final_color=final_color/25;
            h.set(t,material,normal,final_color);
        }
        else
            h.set(t,material,normal);
        return true;
    }

protected:
    float d;
    Vector3f normal;
    Material* material;
    Image *image;
    bool useTexture;
};

class IntersectBox: public Plane {
public:
    IntersectBox(const Vector3f &normal, float d, Material *m = nullptr):Plane(normal,d,m){
    }

    bool intersect_with_box(const Ray &r, Hit &h, float tmin) {
        Vector3f o=r.getOrigin();
        Vector3f dir=r.getDirection();
        dir.normalize();
        float cos=Vector3f::dot(normal,dir);
        if(fabs(cos)<1e-4) 
            return false;
        float t=(d-Vector3f::dot(normal,o))/cos;
        h.set(t,material,normal);
        return true;
    }

};

#endif //PLANE_H
		

