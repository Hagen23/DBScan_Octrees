/**
*@class OctreeGenerator
*Creates an octree from the point cloud data provided.
*
* @author  Jonathan Langford
*
*
*/

#ifndef OCTREE_GENERATOR_H
#define OCTREE_GENERATOR_H

//#include <pcl/point_cloud.h>
//#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>

#include "HTRBasicDataStructures.h"

using std::vector;
using std::string;

namespace htr{

	class OctreeGenerator{

	public:

		typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
		typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeXYZSearch;
		typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNode LeafNode;
		typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator LeafNodeIterator;
		typedef htr::Point3D Point3D;

		struct Voxel{
			Point3D position;
			float size;
		};

		OctreeGenerator();
		~OctreeGenerator();

		inline CloudXYZ::Ptr getCloud(){return cloud;}
		inline pcl::PointXYZ getCloudCentroid(){return cloudCentroid;}
		inline vector<Voxel>& getVoxels(){return octreeVoxels;}
		inline vector<Point3D>& getCentroids(){return octreeCentroids;}
		inline OctreeXYZSearch::Ptr getOctree() {return octree_p;}

		void initRandomCloud(const float width,const float height,const float depth,const int numOfPoints);
		void initCloudFromFile(string fileName);

        void readCloudFromFile(const char* filename);

		template <typename T>
		void initCloudFromVector(const vector<T>& points);

		void initOctree(const int resolution);

		void extractPointsAtLevel(const int depth);
		//void extractAveragesAtLevel(int depth);
		void stepExtractionLevel(const int step);

	private:
		unsigned int currentExtractionLevel;
		CloudXYZ::Ptr cloud;
		OctreeXYZSearch::Ptr octree_p;

        pcl::PointXYZ cloudCentroid;

		vector<Voxel> octreeVoxels;
		vector<Point3D> octreeCentroids;

		void calculateCloudCentroid();
	};

	///Initializes pcl's cloud data structure from a vector of any type containing x, y, and z member variables.
	///@param[in] points The input data vector.
	template <typename T>
	void OctreeGenerator::initCloudFromVector(const vector<T>& points){
		//Note: Width and Height are only used to store the cloud as an image.
		//Source width and height can be used instead of a linear representation.
		cloud->width = points.size();
		cloud->height = 1;

		cloud->points.resize(cloud->width * cloud->height);

		for (size_t i = 0; i < cloud->points.size (); ++i){
			cloud->points[i].x = points[i].x;
			cloud->points[i].y = points[i].y;
			cloud->points[i].z = points[i].z;
		}
        calculateCloudCentroid();
	}

}

#endif
