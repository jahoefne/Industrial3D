//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#include <string.h>
#include "PointCloud.h"

#include "K3DTree.h"

#include <climits>
#include <cmath>


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

Point3D* getcolor(Point3D* green, Point3D* yellow, Point3D* red,Point3D* final,float interpolation_factor)
;

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

// This function smoothes a given PointCloud within a certain radius and gives a color to the point

PointCloud* PointCloud::smooth(double radius)
{
    vector<Point3D *> neighbors;
    vector<Point3D> smoothPoints = *(new vector<Point3D>());
    Point3D weightedPoint;

    std::vector<double> colordistances;
    double distance;
    double distancecolor;
    long Numpoints = points.size();
    for(int i = 0; i < Numpoints; i++)
    {
      //  std::vector<double> distances;
        neighbors = *(kdTree->findRadiusNeighbors(&(points[i]),radius));

        double weight = 0;
        double weightsum=0;
        double sum =0;
        weightedPoint = *(new Point3D(0,0,0));

        for(unsigned int n= 0; n < neighbors.size(); n++)
        {

              distance = neighbors[n]->sqDistance3d(&(points[i])); // squared distance
              sum+=distance;
          //    distances.push_back(distance);
           // weight = exp(-distance/radius); // calculate weight from distance
           // weightsum+=weight;

           // weightedPoint.x += weight*neighbors[n]->x; // apply weight to the point
           // weightedPoint.y += weight*neighbors[n]->y;
           // weightedPoint.z += weight*neighbors[n]->z;

        }

        for(unsigned int n= 0; n < neighbors.size(); n++)
        {
            distance = neighbors[n]->sqDistance3d(&(points[i])); // squared distance
            weight = distance/sum;

            weightedPoint.x += weight*neighbors[n]->x; // apply weight to the point
            weightedPoint.y += weight*neighbors[n]->y;
            weightedPoint.z += weight*neighbors[n]->z;

        }

   //     for(unsigned int i= 0; i < distances.size(); i++)
    //    {
    //        weight = distances[i]/sum;
    //        weightedPoint.x += weight*neighbors[i]->x; // apply weight to the point
    //        weightedPoint.y += weight*neighbors[i]->y;
    //        weightedPoint.z += weight*neighbors[i]->z;
    //    }

     //   weightedPoint.x = weightedPoint.x/weightsum; // normalize weighted point
     //   weightedPoint.y = weightedPoint.y/weightsum;
     //   weightedPoint.z = weightedPoint.z/weightsum;

        distancecolor = weightedPoint.distance3d(&(points[i])); // compute distance between original point and smoothed one.
        colordistances.push_back(distancecolor);
        smoothPoints.push_back(weightedPoint); // store weighted point
   }
    double maximumdistance=maxAt(colordistances);
    PointCloud *result = (new PointCloud());
    result->points=smoothPoints;

    long Numpoints2 = smoothPoints.size();
    for(int i = 0; i < Numpoints2; i++)
    {
      double value= colordistances[i]/maximumdistance;  //value between 0 and 1.

        Point3D*  greenpoint = (new Point3D(0,0,0));
        greenpoint->rgbSize(0,1,0,1);
        Point3D*  yellowpoint = (new Point3D(0,0,0));
        yellowpoint->rgbSize(1,1,0,1);
        Point3D*  redpoint = (new Point3D(0,0,0));
        redpoint->rgbSize(1,0,0,1);
        Point3D* finalcolor = (new Point3D(0,0,0));

        finalcolor= getcolor(greenpoint,yellowpoint,redpoint,finalcolor,value);
        printf("%f %f %f\n", finalcolor->r, finalcolor->g, finalcolor->b);
        smoothPoints[i].rgbSize(finalcolor->r,finalcolor->b,finalcolor->g,1);

    }
    result->points=smoothPoints;
    return result;
}

double PointCloud::maxAt(std::vector<double>& vector_name) {
    double max =0;
    for (auto val : vector_name) {
        if (max < val) max = val;
    }
    return max;
}


Point3D* PointCloud::getcolor(Point3D* green, Point3D* yellow, Point3D* red,Point3D* final,float interpolation_factor)
{

    const float factor_color1 = std::max(interpolation_factor - 0.5f, 0.0f);
    const float factor_color2 = 0.5f - fabs(0.5f - interpolation_factor);
    const float factor_color3 = std::max(0.5f - interpolation_factor, 0.0f);



    double r = (green->r* factor_color1 +
                yellow->r*factor_color2 +
                red->r* factor_color3) * 2.0f;

    double g = (green->g * factor_color1 +
                yellow->g * factor_color2 +
                red->g * factor_color3) * 2.0f;

    double b = (green->b * factor_color1 +
                yellow->b * factor_color2 +
                red->b * factor_color3) * 2.0f;
    final->rgbSize(r,g,b,1);


    return(final);
}