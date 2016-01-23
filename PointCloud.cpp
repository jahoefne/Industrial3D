//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#include <string.h>
#include "PointCloud.h"
#include "Algorithms.h"
#include "SVD.h"
#include "K3DTree.h"

#include <climits>
#include <cmath>

Point3D  m_bbmin, m_bbmax;

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
}

/*void computeBBox(const std::vector<Point3D> *points)
{
    //if there are no points we return an empty bounding box
      if (points->empty())
      {
        m_bbmin.x = 0;  m_bbmin.y = 0;  m_bbmin.z = 0;
        m_bbmax.x = 0;  m_bbmax.y = 0;  m_bbmax.z = 0;
        return;
      }

      //We now compute the min and max coordinates for our bounding box
      m_bbmin = points->front(); //initialize min with the first point
      m_bbmax = points->front(); //initialize max with the first point

      for (unsigned int i = 0; i < points->size(); ++i)
      {
        const Point3D pt = points->at(i); //do not copy but get a reference to the i-th point in the vector
        if (pt.x < m_bbmin.x) m_bbmin.x = pt.x;const
        else if (pt.x > m_bbmax.x) m_bbmax.x = pt.x;

        if (pt.y < m_bbmin.y) m_bbmin.y = pt.y;
        else if (pt.y > m_bbmax.y) m_bbmax.y = pt.y;

        if (pt.z < m_bbmin.z) m_bbmin.z = pt.z;
        else if (pt.z > m_bbmax.z) m_bbmax.z = pt.z;
      }

      //check how many points we have read from file
      std::cout << "point vector now contains: " << points->size() << " points" << std::endl;

      if (points->empty())
      {
        std::cout << "ERROR: no points to show...(press enter to exit)" << std::endl;
        getc(stdin);
        return;
      }

      //m_sceneCenter = (m_bbmax + m_bbmin) * 0.5;
     // m_sceneRadius = distance3d(m_sceneCenter, m_bbmax);

      std::cout << "\nBounding Box was computed:\n";
      std::cout << "minPoint is: " << m_bbmin.x << "," << m_bbmin.y << "," << m_bbmin.z << std::endl;
      std::cout << "maxPoint is: " << m_bbmax.x << "," << m_bbmax.y << "," << m_bbmax.z << std::endl;

}

*/
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

// Getters for boundaries of the Pointcloud

/*double getMax_x(K3DTree* tree)
{
    double xmax = tree->root->ptrLastPoint->x;
    return xmax;
}
double getMax_y(K3DTree* tree)
{
    double ymax_left = tree->root->left->ptrLastPoint->y;
    double ymax_right = tree->root->right->ptrLastPoint->y;

    double ymax = std::max(ymax_left, ymax_right);
    return ymax;
}
double getMax_z(K3DTree* tree)
{
    double zmax_left1 = tree->root->left->left->ptrLastPoint->z;
    double zmax_left2 = tree->root->left->right->ptrLastPoint->z;

    double zmax_right1 = tree->root->right->left->ptrLastPoint->z;
    double zmax_right2 = tree->root->right->right->ptrLastPoint->z;

    double zmax_left = std::max(zmax_left1,zmax_left2);
    double zmax_right = std::max(zmax_right1,zmax_right2);

    double zmax = std::max(zmax_left, zmax_right);
    return zmax;
}

double getMin_x(K3DTree* tree)
{
    double xmin = tree->root->ptrFirstPoint->x;
    return xmin;
}

double getMin_y(K3DTree* tree)
{
    double ymin_left =  tree->root->left->ptrFirstPoint->y;
    double ymin_right = tree->root->right->ptrFirstPoint->y;

    double ymin = std::max(ymin_left, ymin_right);
    return ymin;
}

double getMin_z(K3DTree* tree)
{
    double zmin_left1 = tree->root->left->left->ptrFirstPoint->z;
    double zmin_left2 = tree->root->left->right->ptrFirstPoint->z;
    double zmin_right1 = tree->root->right->left->ptrFirstPoint->z;
    double zmin_right2 = tree->root->right->right->ptrFirstPoint->z;

    double zmin_left = std::max(zmin_left1,zmin_left2);
    double zmin_right = std::max(zmin_right1, zmin_right2);

    double zmin = std::max(zmin_left, zmin_right);
    return zmin;
}
*/
/*vector<Point3D> getBounds(K3DTree tree)
{





    vector<double> *xbounds = new vector<double>(xmin, xmax);
    vector<double> *ybounds = new vector<double>(ymin, ymax);
    vector<double> *zbounds = new vector<double>(zmin, zmax);

    /*vector<double> bounds = new vector<double>();
    bounds.push_back(xmin);
    bounds.push_back(xmax);
    bounds.push_back(ybounds);
    bounds.push_back(zbounds);

    return bounds;

}*/

#define SAMPLE_SIZE  20

Point3D* getcolor(Point3D* green, Point3D* yellow, Point3D* red,Point3D* final,float interpolation_factor)
;

void PointCloud::alignTo(PointCloud *cloud) {

    vector<Point3D *> vectorDiffs;
    long pointCount = points.size();
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        Point3D* pt = &points[rand() % pointCount];
      //  if (vectorDiffs.size() % 1000 == 0) printf("\n%ld", vectorDiffs.size());
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

void PointCloud::thinning(double radius)
{

    vector<Point3D *> neighbors;

    long Numpoints = points.size();

    for(int i = 0; i < Numpoints; i++)
    {
        if (points[i].ignore == true ) {
            //Do nothing
        }else{
            neighbors = *(kdTree->findRadiusNeighbors(&(points[i]),radius));
            for(unsigned int n= 0; n < neighbors.size(); n++)
            {
           //     if (points[i].x = neighbors[n]){

            //    }
                neighbors[n]->ignore=true;

            }
            points[i].ignore = false;
        }
    }

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
       // printf("%f %f %f\n", finalcolor->r, finalcolor->g, finalcolor->b);
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
