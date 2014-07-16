/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef CLUSTEREXTRACTION_HPP_
#define CLUSTEREXTRACTION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace Processors {
namespace ClusterExtraction {

/*!
 * \class ClusterExtraction
 * \brief ClusterExtraction processor class.
 *
 * ClusterExtraction processor.
 */
class ClusterExtraction: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ClusterExtraction(const std::string & name = "ClusterExtraction");

	/*!
	 * Destructor
	 */
	virtual ~ClusterExtraction();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


// Input data streams

		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_pcl;
		
		Base::DataStreamOut<std::vector<pcl::PointIndices> > out_indices;
		Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_clusters;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_the_biggest_cluster;

// Output data streams

	// Handlers
	Base::EventHandler2 h_extract;
	
	// Handlers
	void extract();
	
	Base::Property<float> clusterTolerance;
	Base::Property<int> minClusterSize;
	Base::Property<int> maxClusterSize;

};

} //: namespace ClusterExtraction
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ClusterExtraction", Processors::ClusterExtraction::ClusterExtraction)

#endif /* CLUSTEREXTRACTION_HPP_ */
