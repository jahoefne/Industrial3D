//
// Created by moe on 06/11/15.
//

#ifndef INDUSTRIAL3D_K3DTREE_H
#define INDUSTRIAL3D_K3DTREE_H

#include "Point3D.h"
#include <algorithm>
#include "PointCloud.h"

class KDNode {
public:
    double median;
    KDNode *left;
    KDNode *right;
    Point3D *ptrFirstPoint;
    Point3D *ptrLastPoint;
};

class K3DTree {
private:
    KDNode *root;

    /** Recursive construction method */
    KDNode *recConstruct(Point3D *begin, Point3D *end, int depth) {
        unsigned long numPoints = (end - begin);
        unsigned long medianPosition = numPoints / 2;
        double medianValue = 0;
        int currentDimension = depth % 3;

        if (currentDimension == 0) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.x > b.x; });
            medianValue = (begin + medianPosition)->x;
        } else if (currentDimension == 1) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.y > b.y; });
            medianValue = (begin + medianPosition)->y;
        } else if (currentDimension == 2) {
            std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.z > b.z; });
            medianValue = (begin + medianPosition)->z;
        }

        KDNode *child = new KDNode;
        child->median = medianValue;
        child->ptrFirstPoint = begin;
        child->ptrLastPoint = end;

        if (numPoints > 1) {
            child->left = this->recConstruct(begin, begin + medianPosition, depth + 1);
            child->right = this->recConstruct(begin + medianPosition, end, depth + 1);
        }

        return child;
    }

    // created by Marleen on 11/11/2015

    // This function only returns nodes right now! It should be changed, so it returns point3D!
    KDNode* regSearch(Point3D *center, KDNode *currentNode, int radius){

        if(currentNode->left == NULL && currentNode->right == NULL) {
            // if the node is a leaf test if it is in range
            if (currentNode->ptrFirstPoint->distanceTo(*center) <= radius) {
                return currentNode;
            }
            else return NULL;
        }
        else if(currentNode->left->ptrFirstPoint->distanceTo(*center)<= radius &&
                currentNode->left->ptrLastPoint->distanceTo(*center) <=radius)
        {// return Subtree of left child
            return currentNode->left;
        }
        else if(currentNode->left->ptrLastPoint->distanceTo(*center) <= radius)// else if region of left child intersects range)
        {// recursion on subtree containing left child
            regSearch(center, currentNode->left, radius);
        }
        else if(currentNode->right->ptrFirstPoint->distanceTo(*center) <= radius &&
                currentNode->right->ptrLastPoint->distanceTo(*center) <= radius)
        {
            //return subtree of right child
            return currentNode->right;
        }
        else if(currentNode->right->ptrFirstPoint->distanceTo(*center) <= radius){
            // else if region of right child intersects range
            //recursion on subtree containing right child
            regSearch(center, currentNode->right, radius);}

    }

public:


    /** Constructor */
    K3DTree(PointCloud *cloud) {
        printf("Building KD Tree \n");
        this->root = this->recConstruct(&cloud->points[0], &cloud->points[cloud->points.size()], 0);
    }
};

// Was going to use this, but ultimately didn't use it.
class Bounds {
public:
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;

    Bounds(Point3D *center, int radius) {
        double xmin = center->x - radius;
        double xmax = center->x + radius;
        double ymin = center->y - radius;
        double ymax = center->y + radius;
        double zmin = center->z - radius;
        double zmax = center->z + radius;
    }
};

#endif //INDUSTRIAL3D_K3DTREE_H
