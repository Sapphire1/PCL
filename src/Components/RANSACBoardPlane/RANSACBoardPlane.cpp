/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "RANSACBoardPlane.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/extract_indices.h>

namespace Processors {
namespace RANSACBoardPlane {

RANSACBoardPlane::RANSACBoardPlane(const std::string & name) :
		Base::Component(name)  {

}

RANSACBoardPlane::~RANSACBoardPlane() {
}

void RANSACBoardPlane::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_pcl", &in_pcl);
registerStream("out_outliers", &out_outliers);
registerStream("out_inliers", &out_inliers);
	// Register handlers
	h_ransac.setup(boost::bind(&RANSACBoardPlane::ransac, this));
	registerHandler("ransac", &h_ransac);
	addDependency("ransac", &in_pcl);

}

bool RANSACBoardPlane::onInit() {

	return true;
}

bool RANSACBoardPlane::onFinish() {
	return true;
}

bool RANSACBoardPlane::onStop() {
	return true;
}

bool RANSACBoardPlane::onStart() {
	return true;
}

void RANSACBoardPlane::ransac() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_pcl.read();
	
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  Eigen::Vector3f v = Eigen::Vector3f(0.0177964, -0.81373, 0.580971);  
  seg.setAxis(v);
  seg.setEpsAngle(90);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    //PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    cout<<"Could not estimate a planar model for the given dataset."<<endl;
  }
//info
  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cout << "Model inliers: " << inliers->indices.size () << std::endl;
//  for (size_t i = 0; i < inliers->indices.size (); ++i)
//    std::cout << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
//                                               << cloud->points[inliers->indices[i]].y << " "
//                                               << cloud->points[inliers->indices[i]].z << std::endl;	
//////////////////////////

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    extract.filter (*cloud_inliers);
    //std::cout << "PointCloud representing the planar component: " << cloud_inliers->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_outliers);
    //*cloud_filtered = *cloud_f;



out_outliers.write(cloud_outliers);
out_inliers.write(cloud_inliers);
}



} //: namespace RANSACBoardPlane
} //: namespace Processors
