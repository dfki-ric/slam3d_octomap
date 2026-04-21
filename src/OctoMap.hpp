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
		bool remove_dynamic_objects();

	protected:
		octomap::OcTree mOcTree;
		OctoMapConfiguration mConfig;

		Clock* mClock;
		Logger* mLogger;
		Graph* mGraph;
	};
}
