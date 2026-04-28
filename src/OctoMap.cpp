#include "OctoMap.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

using namespace slam3d;

OctoMap::OctoMap(const OctoMapConfiguration &conf, Clock* c, Logger* l, Graph* g)
: mOcTree(conf.resolution), mConfig(conf), mClock(c), mLogger(l), mGraph(g)
{
	mOcTree.setOccupancyThres(conf.occupancyThres);
	mOcTree.setProbHit(conf.probHit);
	mOcTree.setProbMiss(conf.probMiss);
	mOcTree.setClampingThresMin(conf.clampingThresMin);
	mOcTree.setClampingThresMax(conf.clampingThresMax);
}

void OctoMap::addMeasurement(PointCloudMeasurement::Ptr scan, const Transform& pose)
{
	PointCloud::Ptr tempCloud(new PointCloud);
	Transform sensor = pose * scan->getSensorPose();
	pcl::transformPointCloud(*(scan->getPointCloud()), *tempCloud, sensor.matrix());

	octomap::Pointcloud octoCloud;
	for(PointCloud::iterator it = tempCloud->begin(); it < tempCloud->end(); ++it)
	{
		octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
	}
	Eigen::Vector3d origin = sensor.translation();
	mOcTree.insertPointCloud(octoCloud, octomap::point3d(origin(0), origin(1), origin(2)), mConfig.rangeMax, true, true);
}

void OctoMap::sendMap()
{
	mOcTree.updateInnerOccupancy();
	mOcTree.writeBinaryConst("slam3d_octomap.bt");
}

bool OctoMap::remove_dynamic_objects()
{
	mLogger->message(INFO, "Requested dynamic object removal.");
	timeval start = mClock->now();
	VertexObjectList vertices = mGraph->getVerticesByType("slam3d::PointCloudMeasurement");
	unsigned deleted = 0;
	
	for(VertexObjectList::iterator v = vertices.begin(); v != vertices.end(); ++v)
	{
		// Cast to PointCloudMeasurement
		PointCloudMeasurement::Ptr m =
			boost::dynamic_pointer_cast<PointCloudMeasurement>(mGraph->getMeasurement(v->index));
		if(!m)
		{
			mLogger->message(WARNING, "Vertex given to remove_dynamic_objects is not a Pointcloud!");
			continue;
		}
		
		// Check each point, if it is in free OctoMap voxel
		deleted += removeDynamicObjectsFromCloud(m->getPointCloud(), v->correctedPose * m->getSensorPose());

	}
	int duration = mClock->now().tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Removed %1% dynamic points in %2% seconds.") % deleted % duration).str());
	return true;
}

unsigned OctoMap::removeDynamicObjectsFromCloud(PointCloud::Ptr cloud, const Transform &cloudOrigin)
{
	unsigned deleted = 0;

	pcl::PointIndices::Ptr toBeErased (new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// for(PointCloud::iterator p = cloud->begin(); p != cloud->end();)
	for(size_t i = 0; i<cloud->size(); ++i)
	{
		Eigen::Vector3d p_tf;
		p_tf[0] = (*cloud)[i].x;
		p_tf[1] = (*cloud)[i].y;
		p_tf[2] = (*cloud)[i].z;
		p_tf = cloudOrigin * p_tf;

		if( isOccupied(p_tf))
		{
			deleted++;
			toBeErased->indices.push_back(i);
		}
	}
	extract.setInputCloud(cloud);
	extract.setIndices(toBeErased);
	extract.setNegative(true);
	extract.filter(*cloud);

	return deleted;
}

bool OctoMap::isOccupied(const Eigen::Vector3d &p_tf) {
	octomap::OcTreeNode* node = mOcTree.search(p_tf[0], p_tf[1], p_tf[2]);
	if (!node) {
		 // if node is not found, report occupied
		 // this is used as a filter, it should not report "free" with no information in the tree
		return true; 
	}
	return mOcTree.isNodeOccupied(node);
}
