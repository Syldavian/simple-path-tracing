#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "image.hpp"
#include "curve.hpp"
#include <tuple>
#include <vector>
#include <cmath>

class RevSurface : public Object3D {

    Curve *pCurve;
    std::vector<CurvePoint> curvePoints;
    Material* material;
    Image *image;
    bool useTexture;
    //for bounding cylinder
    float y_min=1e9,y_max=-1e9;
    float radius=0;
    //discretize sampling
    float samps=1000.0;
    float ttsamps;

public:
    RevSurface(Curve *pCurve, Material* material, Image *img=nullptr, bool tex=false) : 
    pCurve(pCurve), Object3D(material), material(material),image(img),useTexture(tex) {
        // Check flat.
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
            //construct bounding cylinder
            if(cp.y()>y_max) 
                y_max=cp.y();
            if(cp.y()<y_min) 
                y_min=cp.y();
            if(fabs(cp.x())>radius) 
                radius = fabs(cp.x());
        }
        pCurve->discretize(samps,curvePoints);
        int n_k_1=pCurve->getControlsSize()-3;
        ttsamps=n_k_1*samps;
    }

    ~RevSurface() override {
        delete pCurve;
    }

    Vector2f calcV(float t){
        int i=int(ttsamps*t);
        float dis1=ttsamps*t-i;
        double dis2=1-dis1;
        Vector3f weighted_avg;
        if(i<ttsamps-1)
            weighted_avg=dis2*curvePoints[i].V+dis1*curvePoints[i+1].V;
        else
            weighted_avg=curvePoints[i].V;
        return weighted_avg.xy();
    }

    Vector2f calcT(float t){
        int i=int(ttsamps*t);
        float dis1=ttsamps*t-i;
        double dis2=1-dis1;
        Vector3f weighted_avg;
        if(i<ttsamps-1)
            weighted_avg=dis2*curvePoints[i].T+dis1*curvePoints[i+1].T;
        else
            weighted_avg=curvePoints[i].T;
        return weighted_avg.xy();
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        // (PA3 optional TODO): implement this for the ray-tracing routine using G-N iteration.
        Vector2f planeDir=r.getDirection().xz();
        float ydir=r.getDirection().y();
        float t_ratio=planeDir.abs();
        planeDir.normalize();
        Vector2f planeOrig=r.getOrigin().xz();
        float yorig=r.getOrigin().y();
        float t_proj=Vector2f::dot(-planeOrig,planeDir);
        float t_prime_squared=radius*radius-planeOrig.absSquared()+t_proj*t_proj;
        float t_prime=sqrt(t_prime_squared);
        float t_approx;

        if(yorig>y_min&&yorig<y_max){
            if(planeOrig.abs()<=radius){
                t_approx=(t_proj+t_prime)/t_ratio;
            }
            else{
                if(t_proj<=0||t_prime_squared<=0)
                    return false;
                t_approx=(t_proj-t_prime)/t_ratio;
            }
            if(t_approx>h.getT()||t_approx<tmin)
                return false;
            Vector3f inters=r.pointAtParameter(t_approx);
            if(inters.y()<y_min||inters.y()>y_max)
                return false;
        }
        else if(yorig>=y_max){
            if(ydir>=0)
                return false;
            t_approx=(y_max-yorig)/ydir;
            Vector3f inters=r.pointAtParameter(t_approx);
            if(t_approx>h.getT()||t_approx<tmin)
                return false;
            if(inters.xz().abs()>radius){ 
                if(planeOrig.abs()<=radius){
                    t_approx=(t_proj+t_prime)/t_ratio;
                }
                else{
                    if(t_proj<=0||t_prime_squared<=0)
                        return false;
                    t_approx=(t_proj-t_prime)/t_ratio;
                }
                if(t_approx>h.getT()||t_approx<tmin)
                    return false;
                inters=r.pointAtParameter(t_approx);
                if(inters.y()<y_min||inters.y()>y_max)
                    return false;
            }
        }
        else if(yorig<=y_min){
            if(ydir<=0)
                return false;
            t_approx=(y_min-yorig)/ydir;
            Vector3f inters=r.pointAtParameter(t_approx);
            if(t_approx>h.getT()||t_approx<tmin)
                return false;
            if(inters.xz().abs()>radius){ 
                if(planeOrig.abs()<=radius){
                    t_approx=(t_proj+t_prime)/t_ratio;
                }
                else{
                    if(t_proj<=0||t_prime_squared<=0)
                        return false;
                    t_approx=(t_proj-t_prime)/t_ratio;
                }
                if(t_approx>h.getT()||t_approx<tmin)
                    return false;
                inters=r.pointAtParameter(t_approx);
                if(inters.y()<y_min||inters.y()>y_max)
                    return false;
            }
        }
        else{
            return false;
        }
            
        Vector2f intersOnPlane=planeOrig+planeDir*(t_approx*t_ratio);
        float theta=atan(intersOnPlane.y()/intersOnPlane.x());
        if(intersOnPlane.x()<0)
            theta+=M_PI;
        Vector3f inters=r.pointAtParameter(t_approx);
        float yinters=inters.y();
        if(inters.y()<y_min)
            yinters=y_min;
        else if(inters.y()>y_max)
            yinters=y_max;
        float t=(y_max-yinters)/(y_max-y_min);
        if(t>=1)
            t=1-1e-4; 
        else if(t<=0)
            t=1e-4;

        int max_iter=10;
        bool na=false;
        while(!na){
            max_iter--;
            if(max_iter<0)
                break;
            Vector2f Vt=calcV(t);
            Vector2f Tt=calcT(t);
            Vector3f point_on_curve=Vector3f(Vt.x()*cos(theta),Vt.y(),Vt.x()*sin(theta));
            Vector3f err=point_on_curve-r.pointAtParameter(t_approx);
            if(err.length()<0.001)
                break;
            Vector3f partial_t=Vector3f(Tt.x()*cos(theta),Tt.y(),Tt.x()*sin(theta));
            Vector3f partial_theta=Vector3f(-sin(theta)*Vt.x(),0,cos(theta)*Vt.x());
            Vector3f partial_l=-r.getDirection();
            Matrix3f dF=Matrix3f(partial_t,partial_theta,partial_l,true);
            Vector3f X=Vector3f(t,theta,t_approx);
            Vector3f dX=dF.inverse()*err;
            if(dX.length()>1)
                dX.normalize();
            Vector3f newX=X-dX;
            t=newX.x(); 
            theta=newX.y(); 
            t_approx=newX.z();
            if(t>0&&t<1)
                ;
            else if(t>=1) 
                t=1-1e-5;
            else if(t<=0) 
                t=1e-5;
            else 
                na=true;
            if(fabs(theta)>=1e4&&fabs(t_approx)<1e6)
                na=true;
        } 
        if(max_iter<=0||na)
            return false;
        if(t_approx<tmin||t_approx>h.getT())
            return false;

        Quat4f rotation;
        rotation.setAxisAngle(-theta,Vector3f::UP);
        Vector3f pNormal=Vector3f::cross(Vector3f(calcT(t),0), -Vector3f::FORWARD).normalized();
        Vector3f normal=Matrix3f::rotation(rotation)*pNormal;
        if(useTexture){
            float unit_theta=theta/(2*M_PI);
            if(unit_theta>=0)
                unit_theta-=int(unit_theta);
            else 
                unit_theta-=int(unit_theta)-1;
            int u=unit_theta*image->Width();
            int v=t*image->Height();
            Vector3f color=image->GetPixel(u,v);
            h.set(t_approx,material,normal,color);
        }
        else
            h.set(t_approx,material,normal);
        return true;
    }

};

#endif //REVSURFACE_HPP
