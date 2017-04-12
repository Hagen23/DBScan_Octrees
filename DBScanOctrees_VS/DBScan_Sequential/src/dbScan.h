/**
*@file dbScan.h
*DBscan algorithm to cluster 3D points. Sequential version.
*/

#ifndef DBSCAN
#define DBSCAN

#include <vector>
#include <algorithm>
#include "HTRBasicDataStructures.h"
#include <omp.h>

namespace dbScanSpace
{
	/// Searches the space surrounding a point for additional points to consider for clustering.
	///@param[in]	keypoints		Vector of points to be clustered.
	///@param[in]	keypoint		Pivot point.
	///@param[in]	eps				Radius used to search for new points for the cluster.
	///@param[in]	clustered		A vector of that stores if a keypoint from keypoints has already been clustered.
	///@param[out]	retKeys			Vector that stores the neighbor points of keypoint.
    void regionQuery(vector<htr::Point3D> *keypoints, htr::Point3D *keypoint, float eps,
                     vector<bool> *clustered, vector<int> *retKeys)
    {
        float dist;
        retKeys->clear();

        for(int i = 0; i< keypoints->size(); i++)
        {
            if(!clustered->at(i))
            {
                dist = sqrt(pow((keypoint->x - keypoints->at(i).x),2)+pow((keypoint->y - keypoints->at(i).y),2)+
                                pow((keypoint->z - keypoints->at(i).z),2));

                if(dist <= eps && dist != 0.0f)
                {
                    retKeys->push_back(i);
                }
            }
        }
    }

	/// DBscan algorithm to cluster 3D points. 
	///@param[in]	keypoints		Vector of points to be clustered.
	///@param[in]	eps				Radius used to search for new points for the cluster.
	///@param[in]	minPts			The minimum ammount of points that should be in a cluster.
	///@param[out]	clusters		A vector of clustered points.
	void DBSCAN_keypoints(vector<htr::Point3D> *keypoints, float eps, int minPts, vector<vector<htr::Point3D>> *clusters)
    {
        int noKeys = keypoints->size();
        vector<bool> clustered(noKeys,false);
        vector<int> noise;
        vector<bool> visited(noKeys, false);
        vector<int> neighborPts;
        vector<int> neighborPts_;
        int c;

        c = 0;
        clusters->push_back(vector<htr::Point3D>());

        //for each unvisted point P in dataset keypoints
        for(int i = 0; i < noKeys; i++)
        {
            if(!visited[i])
            {
                //Mark P as visited
                visited[i] = true;
                regionQuery(keypoints,&keypoints->at(i),eps,&clustered, &neighborPts);
                if(neighborPts.size() < minPts)
                    //Mark P as Noise
                    noise.push_back(i);
                else
                {
                    clusters->push_back(vector<htr::Point3D>());
                    //expand cluster
                    // add P to cluster c
                    clustered[i] = true;
                    clusters->at(c).push_back(keypoints->at(i));
                    //for each point P' in neighborPts
                    // Expand cluster
                    for(int j = 0; j < neighborPts.size(); j++)
                    {
                        //if P' is not visited
                        if(!visited[neighborPts[j]])
                        {
                            //Mark P' as visited
                            visited[neighborPts[j]] = true;
                            regionQuery(keypoints,&keypoints->at(neighborPts[j]),eps, &clustered, &neighborPts_);

                            if(neighborPts_.size() >= minPts)
                                neighborPts.insert(neighborPts.end(),neighborPts_.begin(),neighborPts_.end());

                        }
                        // if P' is not yet a member of any cluster
                        // add P' to cluster c
                        if(!clustered[neighborPts[j]])
                        {
                             clustered[neighborPts[j]] = true;
                             clusters->at(c).push_back(keypoints->at(neighborPts[j]));
                        }
                    }
                    c++;
                }

            }
        }
    }
}

#endif
