#include "mesh.hpp"
#include "plane.hpp"
#include "kdtree.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

using namespace std;

int currentDim=0;

bool Mesh::Cmp::operator()(TriangleIndex &a, TriangleIndex &b) const {
    return parent->v[a[0]][currentDim]<parent->v[b[0]][currentDim];
}

int Mesh::buildTree(int l, int r, int d) {
    currentDim=d;
    int x=(l+r)/2;
    sort(t.begin()+l, t.begin()+r+1, Cmp(this));
    for(int i=0; i<3; i++){
        tree[x].maximum[i]=-1e30;
        tree[x].minimum[i]=1e30;
        for (int j=0; j<3; j++){
            tree[x].maximum[i]=max(tree[x].maximum[i],v[t[x][j]][i]);
            tree[x].minimum[i]=min(tree[x].minimum[i],v[t[x][j]][i]);
        }
    }
    if(l<x)
        tree[x].leftchild=buildTree(l,x-1,(d+1)%3);
    if(x<r)
        tree[x].rightchild=buildTree(x+1,r,(d+1)%3);
    if(l<x){
        for(int i=0; i<3; i++){
            tree[x].maximum[i]=max(tree[x].maximum[i],tree[tree[x].leftchild].maximum[i]);
            tree[x].minimum[i]=min(tree[x].minimum[i],tree[tree[x].leftchild].minimum[i]);
        }
    }
    if (x < r) {
        for(int i=0; i<3; i++){
            tree[x].maximum[i]=max(tree[x].maximum[i],tree[tree[x].rightchild].maximum[i]);
            tree[x].minimum[i]=min(tree[x].minimum[i],tree[tree[x].rightchild].minimum[i]);
        }
    }
    return x;
}

bool Mesh::query(int x, const Ray &r, Hit &h, float tmin){
    if (!tree.checkIntersect(x, r, tmin, material))
        return false;
    bool result = false;
    TriangleIndex& triIndex = t[x];
    Triangle triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material);
    triangle.normal = n[x];
    result |= triangle.intersect(r, h, tmin);
    if (tree[x].leftchild) result |= query(tree[x].leftchild, r, h, tmin);
    if (tree[x].rightchild) result |= query(tree[x].rightchild, r, h, tmin);
    return result;
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    return query(tree.root,r,h,tmin);
}

Mesh::Mesh(const char *filename, Material *material, float scale, Vector3f offset) : Object3D(material) {

    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    std::string tok;
    int texID;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            //位置+大小
            vec=vec*scale+offset;
            v.push_back(vec);
        }
        else if (tok == fTok) {
            if (line.find(bslash) != std::string::npos) {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(trig);
            } else {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++) {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(trig);
            }
        } 
        else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }

    cout<<"Build KDTree Starts!"<<endl;
    printf("%d\n", t.size());
    printf("vsize: %d\n", v.size());
    tree.t.resize(t.size());
    tree.root=buildTree(0,t.size()-1,0);
    computeNormal();
    cout<<"Build KDTree Completed!"<<endl;

    f.close();
}

void Mesh::computeNormal() {
    n.resize(t.size());
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}
