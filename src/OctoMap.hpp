#pragma once

#include "OctoMapConfiguration.hpp"

#include <octomap/OcTree.h>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

namespace slam3d
{
	class OctoMap
	{
	public:
		OctoMap(const OctoMapConfiguration &conf, Clock* c, Logger* l, Graph* g);
		
		void addMeasurement(PointCloudMeasurement::Ptr scan, const Transform& pose);
		void sendMap();
		void clear();
		bool remove_dynamic_objects();

		unsigned removeDynamicObjectsFromCloud(PointCloud::Ptr cloud, const Transform &cloudOrigin = Transform::Identity());

		bool isOccupied(const Eigen::Vector3d &p_tf);

		const octomap::OcTree& getOcTree() const { return mOcTree; }

	protected:
		octomap::OcTree mOcTree;
		OctoMapConfiguration mConfig;

		Clock* mClock;
		Logger* mLogger;
		Graph* mGraph;
	};
}
