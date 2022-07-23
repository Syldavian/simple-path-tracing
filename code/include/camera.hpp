#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include "plane.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up);
		this->horizontal.normalize();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point, unsigned short *Xi) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle, float focal, float aperture) : 
            Camera(center, direction, up, imgW, imgH),focal_plane_distance(focal),aperture(aperture){
        // angle is in radian.
        f=height/2/tan(angle/2);
    }

    Ray generateRay(const Vector2f &point, unsigned short *Xi) override {
        Vector3f dir(point.x()-width/2,point.y()-height/2,f);
        dir.normalize();
        Matrix3f R(horizontal,up,direction,true);
        Ray r(center,R*dir);
        if(aperture==0) 
            return r;
        else{
            float rand1=erand48(Xi),rand2=2*M_PI*erand48(Xi);
            Vector3f target((point.x()-width/2)*focal_plane_distance/f,(point.y()-height/2)*focal_plane_distance/f,focal_plane_distance);
            float dx=aperture*rand1*cos(rand2);
            float dy=aperture*rand1*sin(rand2);
            Vector3f directn=(target-Vector3f(dx,dy,0)).normalized();
            return Ray(center+dx*horizontal+dy*up,horizontal*directn.x()+up*directn.y()+direction*directn.z());
        }
    }

protected:
    float f;
    float focal_plane_distance;
    float aperture;
};

#endif //CAMERA_H
