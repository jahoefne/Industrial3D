//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#include <string.h>
#include "PointCloud.h"
#include "K3DTree.h"


void PointCloud::print() {
    std::cout << "Read " << this->points.size();
    for (unsigned long i = 0; i < this->points.size(); i++) {
        Point3D *p = &this->points.at(i);
        std::cout << "(" << p->x << ", " << p->y << ", " << p->z << ")\n";
    }
};

/**
 * Initializes the points vector and the kdtree representation from a .xyz ascii file.
 * Returns 0 if successful and != 0 if an error occured
 * Also calculates the center
 */
int PointCloud::loadPointsFromFile(std::string fileName, float r, float g, float b) {
    ifstream pointFile(fileName, ifstream::in);
    if (pointFile.is_open()) {
        double x, y, z;
        int count = 0;
        while (pointFile >> x && pointFile >> y && pointFile >> z) {

            // Read point from file
            Point3D *point = new Point3D(x, y, z);
            point->r = r;
            point->g = g;
            point->b = b;
            points.push_back(*point);

            // consider point for center calculation
            this->center.translate(point);
            count++;

            // reconsider min/max points denoting bounding box
            this->boundingBoxMin.x = (point->x >= this->boundingBoxMin.x) ? this->boundingBoxMin.x : point->x;
            this->boundingBoxMin.y = (point->y >= this->boundingBoxMin.y) ? this->boundingBoxMin.y : point->y;
            this->boundingBoxMax.y = (point->y <= this->boundingBoxMax.y) ? this->boundingBoxMax.y : point->y;
            this->boundingBoxMax.x = (point->x <= this->boundingBoxMax.x) ? this->boundingBoxMax.x : point->x;
        }

        /** Calculate actual center of point cloud*/
        this->center.x /= count;
        this->center.y /= count;
        this->center.z /= count;

        this->sceneRadius = this->center.distanceTo(this->boundingBoxMax);

        cout << "Read file " << fileName << " it contained " << this->points.size() << "points.\n";
        pointFile.close();

        kdTree = new K3DTree(&this->points); // Init Kd-Tree
        return 0;
    } else {
        std::cout << "Can't read file!" << strerror(errno);;
    }
    return -1;
};

void PointCloud::translate(Point3D *point) {
    for_each(points.begin(), points.end(), [&](Point3D pt) {
        pt.translate(point);
    });

    for (Point3D &pt : points) {
        // printf("\nOrigin: %lf, %lf, %lf", pt.x,pt.y,pt.z);
        pt.translate(point);
        // printf("\n\tTranslated for: %lf, %lf, %lf", pt.x,pt.y,pt.z);
    }
    kdTree = new K3DTree(&this->points); // Init Kd-Tree
}

#define SAMPLE_SIZE  20

void PointCloud::alignTo(PointCloud *cloud) {
    vector<Point3D *> vectorDiffs;
    long pointCount = points.size();
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        Point3D* pt = &points[rand() % pointCount];
        if (vectorDiffs.size() % 1000 == 0) printf("\n%ld", vectorDiffs.size());
        Point3D *n = cloud->kdTree->closestNeighbour(pt);
        pt->rgbSize(1,1,0,2);
        pt->rgbSize(0,0,1,2);
        vectorDiffs.push_back(pt->diffVector(n));
    }

    Point3D *avgVec = new Point3D();
    for (Point3D *pt : vectorDiffs) {
        avgVec->translate(pt);
    }

    //printf("AvgVec: %f %f %f\n", avgVec->x, avgVec->y, avgVec->z);
    avgVec->x = -(avgVec->x / (double) vectorDiffs.size());
    avgVec->y = -(avgVec->y / (double) vectorDiffs.size());
    avgVec->z = -(avgVec->z / (double) vectorDiffs.size());

  //  printf("\tEstimated Translation necessary: %f %f %f\n", avgVec->x, avgVec->y, avgVec->z);
    this->translate(avgVec);
    fflush(stdout);
}

// This function smoothes a given PointCloud within a certain radius

PointCloud* PointCloud::smooth(double radius)
{
    vector<Point3D *> neighbors;
    vector<Point3D> smoothPoints = *(new vector<Point3D>());
    Point3D weightedPoint;

    double distance;

    long Numpoints = points.size();
    for(int i = 0; i < Numpoints; i++)
    {
        neighbors = *(kdTree->findRadiusNeighbors(&(points[i]),radius));

        double weight = 0;
        double weightsum=0;
        weightedPoint = *(new Point3D());

        for(unsigned int n= 0; n < neighbors.size(); n++)
        {
            distance = neighbors[n]->sqDistance3d(&(points[i])); // squared distance
            weight = exp(-distance/radius); // calculate weight from distance
            weightsum+=weight;

            weightedPoint.x += weight*neighbors[n]->x; // apply weight to the point
            weightedPoint.y += weight*neighbors[n]->y;
            weightedPoint.z += weight*neighbors[n]->z;

        }

        weightedPoint.x = weightedPoint.x/weightsum; // normalize weighted point
        weightedPoint.y = weightedPoint.y/weightsum;
        weightedPoint.z = weightedPoint.z/weightsum;

        smoothPoints.push_back(weightedPoint); // store weighted point

   }
    PointCloud *result = (new PointCloud());
    result->points=smoothPoints;

    return result;
}


