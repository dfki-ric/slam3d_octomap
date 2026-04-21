#include "OctoMap.hpp"

#include <pcl/common/transforms.h>

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
		PointCloud::Ptr cloud = m->getPointCloud();
		for(PointCloud::iterator p = cloud->begin(); p != cloud->end();)
		{
			Eigen::Vector3d p_tf;
			p_tf[0] = p->x;
			p_tf[1] = p->y;
			p_tf[2] = p->z;
			p_tf = v->correctedPose * m->getSensorPose() * p_tf;
			octomap::OcTreeNode* node = mOcTree.search(p_tf[0], p_tf[1], p_tf[2]);
			if(node && !mOcTree.isNodeOccupied(node))
			{
				deleted++;
				p = cloud->erase(p);
			}else
			{
				p++;
			}
		}
	}
	int duration = mClock->now().tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Removed %1% dynamic points in %2% seconds.") % deleted % duration).str());
	return true;
}
