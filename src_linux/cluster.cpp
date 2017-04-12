#include "cluster.h"

namespace dbScanSpace
{
    cluster::cluster()
    {
        visited = false;
    }

    void cluster::calculateCentroid()
    {
        if(clusterPoints.size() > 0)
        {
            //for(pcl::PointXYZ point:clusterPoints)
            //{
            //    centroid3D.x = centroid.x += point.x;
            //    centroid3D.y = centroid.y += point.y;
            //    centroid3D.z = centroid.z += point.z;
            //}
            //centroid3D.x = centroid.x /= clusterPoints.size();
            //centroid3D.y = centroid.y /= clusterPoints.size();
            //centroid3D.z = centroid.z /= clusterPoints.size();
		    for(auto& point:clusterPoints)
			{
                centroid.x += point.x;
                centroid.y += point.y;
                centroid.z += point.z;
            }
            centroid3D.x = centroid.x /= clusterPoints.size();
            centroid3D.y = centroid.y /= clusterPoints.size();
            centroid3D.z = centroid.z /= clusterPoints.size();
        }
    }

    void cluster::toPoint3D()
    {
        if(clusterPoints.size() > 0)
        {
            for(pcl::PointXYZ point:clusterPoints)
            {
                htr::Point3D pointAux;
                pointAux.x = point.x;
                pointAux.y = point.y;
                pointAux.z = point.z;
                clusterPoints3D.push_back(pointAux);
            }
        }
    }
}
