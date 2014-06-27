/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef RANSACBOARDPLANE_HPP_
#define RANSACBOARDPLANE_HPP_

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
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <Types/MatrixTranslator.hpp>
#include <opencv2/core/core.hpp>



namespace Processors {
namespace RANSACBoardPlane {

/*!
 * \class RANSACBoardPlane
 * \brief RANSACBoardPlane processor class.
 *
 * RANSACBoardPlane processor.
 */
class RANSACBoardPlane: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RANSACBoardPlane(const std::string & name = "RANSACBoardPlane");

	/*!
	 * Destructor
	 */
	virtual ~RANSACBoardPlane();

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

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_pcl;

// Output data streams

	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_outliers;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_inliers;
	// Handlers
	Base::EventHandler2 h_ransac;

	
	// Handlers
	void ransac();

};

} //: namespace RANSACBoardPlane
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RANSACBoardPlane", Processors::RANSACBoardPlane::RANSACBoardPlane)

#endif /* RANSACBOARDPLANE_HPP_ */
