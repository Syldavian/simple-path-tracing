#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>
#include <cmath>
#include <tuple>

#include <algorithm>

// (PA3): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.
class Bernstein{
protected:
    int n;
    int k;
    std::vector<float> t;
    std::vector<float> tpad;
public:
    Bernstein(int n,int k,std::vector<float> t):n(n),k(k),t(t){
        tpad=t;
        for(int i=0;i<k;i++)
            tpad.push_back(t.back());
    }

    static std::vector<float> bezier_knot(int k){
        int _n=k+1;
        std::vector<float> knot;
        for(int i=0;i<_n;i++)
            knot.push_back(0);
        for(int i=0;i<_n;i++)
            knot.push_back(1);
        return knot;
    }

    static std::vector<float> bspline_knot(int n,int k){
        std::vector<float> knot;
        for(int i=0;i<n+k+1;i++){
            float x=(float)i/(float)(n+k);
            knot.push_back(x);
        }
        return knot;
    }

    int get_bpos(float mu){
        if(mu<t.front()||mu>t.back())
            throw "Value Error!";
        if(mu==t.front()){
            int bpos=0;
            while(bpos<t.size()&&t[bpos]<=mu)
                bpos++;
            return bpos-1;
        }
        else{
            int bpos=0;
            while(bpos<t.size()&&t[bpos]<mu)
                bpos++;
            if(bpos-1>0)
                return bpos-1;
            else
                return 0;
        }
    }

    std::pair<float, float> get_valid_range(){
        float start_t=t[k];
        float end_t=t[t.size()-k-1];
        return std::make_pair(start_t,end_t);
    }

    int get_point_number(){
        return n-k;
    }

    std::tuple<std::vector<float>,std::vector<float>,int> evaluate(float mu){
        int bpos=get_bpos(mu);
        std::vector<float> s(k+2,0);
        s[s.size()-2]=1;
        std::vector<float> ds(k+1,1);
        for(int p=1;p<k+1;p++){
            for(int ii=k-p;ii<k+1;ii++){
                int i=ii+bpos-k;
                float w1=0,w2=0,dw1=0,dw2=0;
                if(tpad[i+p]==tpad[i]){
                    w1=mu;
                    dw1=1;
                } 
                else{
                    w1=(mu-tpad[i])/(tpad[i+p]-tpad[i]);
                    dw1=1.0f/(tpad[i+p]-tpad[i]);
                }
                if(tpad[i+p+1]==tpad[i+1]){
                    w2=1-mu;
                    dw2=-1;
                }
                else{
                    w2=(tpad[i+p+1]-mu)/(tpad[i+p+1]-tpad[i+1]);
                    dw2=-1.0f/(tpad[i+p+1]-tpad[i+1]);
                }
                if(p==k) 
                    ds[ii]=(dw1*s[ii]+dw2*s[ii+1])*p;
                s[ii]=w1*s[ii]+w2*s[ii+1];
            }
        }
        s.pop_back();
        int lsk=bpos-k;
        int rsk=n-bpos-1;
        if(lsk<0){
            std::vector<float> _s(s.begin()-lsk,s.end());
            std::vector<float> _ds(ds.begin()-lsk,ds.end());
            s=_s;
            ds=_ds;
            lsk=0;
        }
        if(rsk<0){
            std::vector<float> _s(s.begin(),s.end()+rsk);
            std::vector<float> _ds(ds.begin(),ds.end()+rsk);
            s=_s;
            ds=_ds;
        }
        return std::make_tuple(s,ds,lsk);
    }
};

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
public:
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    int getControlsSize(){
        return controls.size();
    }

    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // (PA3): fill in data vector
        std::vector<float> knot=Bernstein::bezier_knot(3);
        Bernstein bernstein(controls.size(),3,knot);
        int num=bernstein.get_point_number()*resolution;
        std::pair<float, float> range=bernstein.get_valid_range();
        for(int i=0;i<=num;i++){
            //f(t) for each t
            Vector3f pt=Vector3f::ZERO;
            Vector3f dpt=Vector3f::ZERO;
            float mu=range.first+((float)i/(float)num)*(range.second-range.first);
            auto basis=bernstein.evaluate(mu);
            std::vector<float> s=std::get<0>(basis);
            std::vector<float> ds=std::get<1>(basis);
            int lsk=std::get<2>(basis);
            for(int j=0;j<s.size();j++){
                //calculate f(t) by accumulating each point's value
                pt+=s[j]*controls[lsk+j];
                dpt+=ds[j]*controls[lsk+j];
            }
            CurvePoint c;
            c.V=pt;
            c.T=dpt;
            data.push_back(c);
        }
    }

protected:

};

class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // (PA3): fill in data vector
        std::vector<float> knot=Bernstein::bspline_knot(controls.size(),3);
        Bernstein bernstein(controls.size(),3,knot);
        int num=bernstein.get_point_number()*resolution;
        std::pair<float, float> range=bernstein.get_valid_range();
        for(int i=0;i<=num;i++){
            //f(t) for each t
            Vector3f pt=Vector3f::ZERO;
            Vector3f dpt=Vector3f::ZERO;
            float mu=range.first+((float)i/(float)num)*(range.second-range.first);
            auto basis=bernstein.evaluate(mu);
            std::vector<float> s=std::get<0>(basis);
            std::vector<float> ds=std::get<1>(basis);
            int lsk=std::get<2>(basis);
            for(int j=0;j<s.size();j++){
                //calculate f(t) by accumulating each point's value
                pt+=s[j]*controls[lsk+j];
                dpt+=ds[j]*controls[lsk+j];
            }
            CurvePoint c;
            c.V=pt;
            c.T=dpt;
            data.push_back(c);
        }
    }

protected:

};

#endif // CURVE_HPP
