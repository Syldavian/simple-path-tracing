#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects) {
        objectGroup.resize(num_objects);
    }

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool isIntersect=false;
        for(auto i:objectGroup){
            if(i->intersect(r,h,tmin))
                isIntersect=true;
        }
        return isIntersect;
    }

    void addObject(int index, Object3D *obj) {
        objectGroup[index]=obj;
    }

    int getGroupSize() {
        return objectGroup.size();
    }

private:
    std::vector<Object3D *> objectGroup;
};

#endif
	
