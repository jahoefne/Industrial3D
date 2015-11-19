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
    KDNode *left = nullptr;
    KDNode *right = nullptr;
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
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.x < b.x; });
                medianValue = (begin + medianPosition)->x;
                break;
            case Y_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.y <  b.y; });
                medianValue = (begin + medianPosition)->y;
                break;
            case Z_AXIS:
                std::sort(begin, end, [](const Point3D &a, const Point3D &b) -> bool { return a.z < b.z; });
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


    Point3D *closestNeighbour(Point3D *tgt) {
        return recClosestNeighbour(tgt, root, X_AXIS);
    }

    std::vector<Point3D*>* findRadiusNeighbors(Point3D* queryPoint, double radius){
        std::vector<Point3D*> *neighbors = new std::vector<Point3D*>();
        printf("Find radius neighbour");
        findRadiusNeighborsRec(root, queryPoint, radius, neighbors, 0);
        return neighbors;
    }

    std::vector<Point3D*>* findRange(Point3D* min, Point3D* max){
        std::vector<Point3D*> *neighbors = new std::vector<Point3D*>();
        printf("Find range neighbour %lf %lf %lf",max->x, max->y, max->z);
        findRangeRec(root, min, max, neighbors, 0);
        return neighbors;
    }

private:

    void findRadiusNeighborsRec(KDNode* node, Point3D* queryPoint, double radius, std::vector<Point3D*>* neighbors, unsigned int depth)
    {
        if (node->left == node->right) // if this is true the node is a leaf
        {
            Point3D* pt = node->ptrFirstPoint;

            // We don't use the expensive sqrt-function but instead compare the squared values
            const double sqDistance = pt->sqDistance3d(queryPoint); // squared distance
            if (sqDistance > (radius*radius)) return; // point outside the sphere
            neighbors->push_back(node->ptrFirstPoint);
        }
        else
        {
            const unsigned int DIM = depth % 3;
            double v_min, v_max;
            if (DIM == 0){
                v_min = queryPoint->x - radius;
                v_max = queryPoint->x + radius;
            }
            else if (DIM == 1){
                v_min = queryPoint->y - radius;
                v_max = queryPoint->y + radius;
            }
            else{
                v_min = queryPoint->z - radius;
                v_max = queryPoint->z + radius;
            }

            if (v_min <= node->median) findRadiusNeighborsRec(node->left , queryPoint, radius, neighbors, depth + 1);
            if (v_max  > node->median) findRadiusNeighborsRec(node->right, queryPoint, radius, neighbors, depth + 1);
        }
    }

    void findRangeRec(const KDNode* node, Point3D* min, Point3D* max, std::vector<Point3D*>* neighbors, unsigned int depth)
    {
        if (node->left == node->right) //if this is true the node is a leaf
        {
            const Point3D& pt=*node->ptrFirstPoint;
            //a final check still must be done
            if (pt.x < min->x || pt.x > max->x) return;
            if (pt.y < min->y || pt.y > max->y) return;
            if (pt.z < min->z || pt.z > max->z) return;

            neighbors->push_back(node->ptrFirstPoint);
        }
        else
        {
            const unsigned int DIM=depth % 3;
            double v_min, v_max;
            if (DIM == X_AXIS){
                v_min=min->x;
                v_max=max->x;
            }
            else if(DIM == Y_AXIS){
                v_min=min->y;
                v_max=max->y;
            }
            else{
                v_min=min->z;
                v_max=max->z;
            }

            if (v_min <= node->median) findRangeRec(node->left, min, max, neighbors, depth+1);
            if (v_max  > node->median) findRangeRec(node->right, min, max, neighbors, depth+1);
        }
    }


    Point3D *recClosestNeighbour(Point3D *tgt, KDNode *curr, int dim) {
        double value = 0;
        switch (dim % 3) {
            case X_AXIS:
                value = tgt->x;
                break;
            case Y_AXIS:
                value = tgt->y;
                break;
            case Z_AXIS:
                value = tgt->z;
                break;
            default:
                printf("Should never happen");
                return NULL;
        }
        if (curr->median > value && curr->left != NULL) {
            recClosestNeighbour(tgt, curr->left, dim + 1);
        } else if (curr->right != NULL) {
            return recClosestNeighbour(tgt, curr->right, dim + 1);
        }
        return curr->ptrFirstPoint;
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
