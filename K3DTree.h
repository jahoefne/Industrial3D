//
// Created by moe on 06/11/15.
//

#ifndef INDUSTRIAL3D_K3DTREE_H
#define INDUSTRIAL3D_K3DTREE_H

#include <algorithm>
#include "PointCloud.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

class KDNode {
public:
    double median;
    KDNode *left;
    KDNode *right;
    Point3D *ptrFirstPoint;
    Point3D *ptrLastPoint;
};

class K3DTree {
public:
    // std::vector<Point3D> points_range;
    KDNode *root;

    /** Constructor */
    K3DTree(std::vector<Point3D> *points) {
        printf("Building KD Tree with %ld Points\n", points->size());
        this->root = this->recConstruct(&points->at(0), &points->at(points->size() - 1), 0);
    }

    /** Recursive construction method */
    KDNode *recConstruct(Point3D *begin, Point3D *end, int depth) {
        unsigned long numPoints = (end - begin);
        unsigned long medianPosition = numPoints / 2;
        double medianValue = 0;

        switch (depth % 3) {
            case X_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.x > b.x; });
                medianValue = (begin + medianPosition)->x;
                break;
            case Y_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.y > b.y; });
                medianValue = (begin + medianPosition)->y;
                break;
            case Z_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.z > b.z; });
                medianValue = (begin + medianPosition)->z;
                break;
            default:
                printf("Can not happen!");
                break;
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


    std::vector<Point3D*>* closestNeighbours(Point3D tgt, int number){
        return NULL;
    }


private:


    std::vector<Point3D*>* recClosestNeighbours(Point3D tgt, int number){
        return NULL;
    }



/*
    //created by Rene
    //reportSubTree, which traverses the subtree rooted at the node and reports all the stored at its leaves.
    //  void reportSubTree(KDNode *currentNode){
    //if v is a leaf
    //     if(currentNode->left == NULL && currentNode->right == NULL) {
    // We display the points in his leaf , last point and first point are the same, so every leaf store a point
    // from the bunny.xyz
    // currentNode->ptrLastPoint->printpoint();
    //         printf("Printing coordinate x %lf \n", currentNode->ptrLastPoint->x);
    //        printf("Printing coordinate y %lf \n", currentNode->ptrLastPoint->y);
    //        printf("Printing coordinate z %lf \n", currentNode->ptrLastPoint->z);
    //    }else{
    //       if(currentNode->right != NULL) {
    //           reportSubTree(currentNode->right);
    //      }
    //Recurse on left subtree
    //       if(currentNode->left != NULL) {
    //           reportSubTree(currentNode->left);
    //       }
//        }
    //    }
    */
/*
    //reportSubTree, which traverses the subtree rooted at the node and reports all the stored at its leaves.
    void reportSubTree(KDNode *currentNode) {
        if (currentNode->left == NULL && currentNode->right == NULL) {
            Point3D *point = new Point3D(currentNode->ptrLastPoint->x, currentNode->ptrLastPoint->y,
                                         currentNode->ptrLastPoint->z);
            points_range.push_back(*point);
            printf("Printing coordinate x %lf \n", currentNode->ptrLastPoint->x);
            printf("Printing coordinate y %lf \n", currentNode->ptrLastPoint->y);
            printf("Printing coordinate z %lf \n", currentNode->ptrLastPoint->z);


        } else {
            if (currentNode->right != NULL) {
                reportSubTree(currentNode->right);

            } else {
                return;
            }
            if (currentNode->left != NULL) {
                reportSubTree(currentNode->left);
            } else {
                return;
            }
        }

    }*/
/*
    // created by Marleen on 11/11/2015 and Rene
    // This function only returns nodes right now! It should be changed, so it returns point3D!
    void regSearch(Point3D *center, KDNode *currentNode, double radius) {

        if (currentNode->left == NULL && currentNode->right == NULL) {
            // if the node is a leaf test if it is in range
            if (currentNode->ptrFirstPoint->distanceTo(*center) <= radius) {
                points_range.push_back(*(currentNode->ptrFirstPoint));
                return;
            } else return;
        }
        else if (currentNode->left->ptrFirstPoint->distanceTo(*center) <= radius &&
                 currentNode->left->ptrLastPoint->distanceTo(*center) <= radius) {// return Subtree of left child
            reportSubTree(currentNode->left);
            return;
        }
        else if (currentNode->left->ptrLastPoint->distanceTo(*center) <=
                 radius)// else if region of left child intersects range)
        {// recursion on subtree containing left child
            regSearch(center, currentNode->left, radius);
        }
        else if (currentNode->right->ptrFirstPoint->distanceTo(*center) <= radius &&
                 currentNode->right->ptrLastPoint->distanceTo(*center) <= radius) {
            //return subtree of right child
            reportSubTree(currentNode->right);
            return;
        }
        else if (currentNode->right->ptrFirstPoint->distanceTo(*center) <= radius) {
            // else if region of right child intersects range
            //recursion on subtree containing right child
            regSearch(center, currentNode->right, radius);
        }
        return;
    }*/



};

#endif //INDUSTRIAL3D_K3DTREE_H
