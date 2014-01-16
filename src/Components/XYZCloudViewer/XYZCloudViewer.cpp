/*!
 * \file
 * \brief Contains methods of the component able to display may XYZ point clouds in one window.
 * \author Tomasz Kornuta [tkornuta@gmail.com]
 */

#include <memory>
#include <string>

#include "XYZCloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace XYZCloudViewer {

XYZCloudViewer::XYZCloudViewer(const std::string & name) :
		Base::Component(name),
		title("title", std::string("XYZ Cloud Viewer")),
		count("count", 1),
		clouds_colours("clouds_colours", cv::Mat(cv::Mat::zeros(1, 3, CV_8UC1)))

{
  registerProperty(title);
  registerProperty(count);
  registerProperty(clouds_colours);

  // Check colours.


//  count.setToolTip("Total number of displayed clouds");
}


XYZCloudViewer::~XYZCloudViewer() {
}

void XYZCloudViewer::prepareInterface() {
	// Register data streams and event handlers depending on the number of clouds.
	Base::EventHandler2 * hand;
	for (int i = 0; i < count; ++i) {
		char id = '0' + i;
		
		// Create datastream for i-th cloud.
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest,
				Base::Synchronization::Mutex> * stream =
				new Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest,
						Base::Synchronization::Mutex>;
		in_clouds.push_back(stream);
		// Register i-th stream.
		registerStream(std::string("in_cloud_xyz") + id,
				(Base::DataStreamInterface*) (in_clouds[i]));

		// Create new handler for i-th cloud.
		hand = new Base::EventHandler2;
		hand->setup(boost::bind(&XYZCloudViewer::on_cloud_xyzN, this, i));
		handlers.push_back(hand);
		registerHandler(std::string("on_cloud_xyz") + id, hand);
		// Add dependency for i-th stream.
		addDependency(std::string("on_cloud_xyz") + id, stream);
	}

	// Register aliases for first handler and streams.
	registerStream("in_cloud_xyz", in_clouds[0]);
	registerHandler("on_cloud_xyz", handlers[0]);


	// Register spin handler.
	h_on_spin.setup(boost::bind(&XYZCloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool XYZCloudViewer::onInit() {
	// Create visualizer.
	viewer = new pcl::visualization::PCLVisualizer (title);
	viewer->initCameraParameters ();
	// Add visible coortinate system.
	viewer->addCoordinateSystem (1.0);

	// Add clouds.
	for (int i = 0; i < count; ++i) {
		char id = '0' + i;
		
		// Create i-th cloud.
		viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), std::string("in_cloud_xyz") + id);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::string("in_cloud_xyz") + id);
		// Set cloud colour depending on the property.
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
			((cv::Mat)clouds_colours).at<int>(i, 0),
			((cv::Mat)clouds_colours).at<int>(i, 1),
			((cv::Mat)clouds_colours).at<int>(i, 2),
			std::string("in_cloud_xyz") + id); 
	}

	return true;
}

bool XYZCloudViewer::onFinish() {
	return true;
}

bool XYZCloudViewer::onStop() {
	return true;
}

bool XYZCloudViewer::onStart() {
	return true;
}

void XYZCloudViewer::on_cloud_xyzN(int n) {
	// Read input cloud from n-th dataport.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_clouds[n]->read();
	char id = '0' + n;
	// Refresh n-th cloud.
	viewer->updatePointCloud<pcl::PointXYZ> (cloud, std::string("in_cloud_xyz") + id);
}

/*void XYZCloudViewer::on_cloud_xyz1() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = in_cloud_xyz1.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud1, "sample cloud1");
}*/


void XYZCloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace XYZCloudViewer
} //: namespace Processors
