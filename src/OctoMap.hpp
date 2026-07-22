#pragma once

#include "OctoMapConfiguration.hpp"

#include <octomap/OcTree.h>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/core/MeasurementStorage.hpp>

namespace slam3d
{
	class OctoMap
	{
	public:
		OctoMap(const OctoMapConfiguration &conf, Clock* c, Logger* l, MeasurementStorage* s);
		
		void addMeasurement(PointCloudMeasurement::Ptr scan, const MetaData& meta, const Transform& pose);
		void sendMap();
		void clear();
		bool remove_dynamic_objects(const VertexObjectList& vertices, PointCloud::Ptr removed = {});

		unsigned removeDynamicObjectsFromCloud(
			PointCloud::Ptr cloud,
			const Transform &cloudOrigin = Transform::Identity(),
			PointCloud::Ptr removed = {});

		bool isOccupied(const Eigen::Vector3d &p_tf);

		const octomap::OcTree& getOcTree() const { return mOcTree; }

	protected:
		octomap::OcTree mOcTree;
		OctoMapConfiguration mConfig;

		Clock* mClock;
		Logger* mLogger;
		MeasurementStorage* mStorage;
	};
}
