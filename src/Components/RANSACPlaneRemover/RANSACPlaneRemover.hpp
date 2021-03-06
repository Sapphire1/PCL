/*!
 * \file
 * \brief 
 * \author Lukasz Zmuda
 */

#ifndef RANSACPLANEREMOVER_HPP_
#define RANSACPLANEREMOVER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


namespace Processors {
namespace RANSACPlaneRemover {

/*!
 * \class RANSACPlaneRemover
 * \brief RANSACPlaneRemover processor class.
 *
 * RANSACPlaneRemover processor.
 */
class RANSACPlaneRemover: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RANSACPlaneRemover(const std::string & name = "RANSACPlane");

	/*!
	 * Destructor
	 */
	virtual ~RANSACPlaneRemover();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();
	
	Base::Property<int> prop_alfa_treshold;
	Base::Property<int> prop_iter_treshold;
	Base::Property<double> prop_HeightLimitMax;
	Base::Property<double> prop_HeightLimitMin;
	Base::Property<double> prop_A1;
	Base::Property<double> prop_B1;
	Base::Property<double> prop_C1;
	Base::Property<double> prop_D1Min;
	Base::Property<double> prop_D1Max;
	
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

	//double alfaTreshold = 30;
	//int iterTreshold = 5;
	

	// Input data streams

	 Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_pcl;

	// Output data streams

	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_outliers;
	//Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>> out_outliers;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_inliers;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  out_object_points;
	// Handlers
	Base::EventHandler2 h_ransac;

	
	// Handlers
	void ransac();

};

} //: namespace RANSACPlaneRemover
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RANSACPlaneRemover", Processors::RANSACPlaneRemover::RANSACPlaneRemover)

#endif /* RANSACPLANEREMOVER_HPP_ */
