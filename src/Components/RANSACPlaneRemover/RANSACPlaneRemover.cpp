/*!
 * \file
 * \brief
 * \author Micha Laszkowski, Lukasz Zmuda
 */

#include <memory>
#include <string>

#include "RANSACPlaneRemover.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace std;
using namespace pcl;


namespace Processors {
namespace RANSACPlaneRemover {
  
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

RANSACPlaneRemover::RANSACPlaneRemover(const std::string & name) :
		Base::Component(name),  
		prop_alfa_treshold("ransacPlane.alfaTreshold", 30, "range"),
		prop_iter_treshold("ransacPlane.iterTreshold", 5, "range")
{
		cout<<"START!!!\n";
		prop_alfa_treshold.addConstraint("1");
		prop_alfa_treshold.addConstraint("90");
		registerProperty(prop_alfa_treshold);
		prop_iter_treshold.addConstraint("0");
		prop_iter_treshold.addConstraint("10");
		registerProperty(prop_iter_treshold);
}

RANSACPlaneRemover::~RANSACPlaneRemover() {
  
}

void RANSACPlaneRemover::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	registerStream("out_outliers", &out_outliers);
	registerStream("out_inliers", &out_inliers);
	registerStream("out_object_points", &out_object_points);
	// Register handlers
	h_ransac.setup(boost::bind(&RANSACPlaneRemover::ransac, this));
	registerHandler("ransac", &h_ransac);
	addDependency("ransac", &in_pcl);

}

bool RANSACPlaneRemover::onInit() {

	return true;
}

bool RANSACPlaneRemover::onFinish() {
	return true;
}

bool RANSACPlaneRemover::onStop() {
	return true;
}

bool RANSACPlaneRemover::onStart() {
	return true;
}

void RANSACPlaneRemover::ransac() {
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_pcl.read();
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  int iter = 0;
  
  //cout<<"Filter\n";
  
  // find proper plane
  //while(!founded & iter<tresholdIter)
  while(iter<=prop_iter_treshold)
  {
	  seg.setInputCloud (cloud);
	  seg.segment (*inliers, *coefficients);
	  if (inliers->indices.size () == 0)
	  {
	       //PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	       cout<<"Could not estimate a planar model for the given dataset."<<endl;
	  }
	  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
					    << coefficients->values[1] << " "
					    << coefficients->values[2] << " " 
					    << coefficients->values[3] << std::endl;
					    
	 // Compare coefficients
	 // 0.038248 -0.814044 0.579542 <- normal vector of board plane
	 // count angle
	 //
					    
	 double A1 = 0.038248;
	 double B1 = -0.814044;
	 double C1 = 0.579542;
	 
	 double A2 = coefficients->values[0];
	 double B2 = coefficients->values[1];
	 double C2 = coefficients->values[2];
	 
	 double licz = fabs(A1*A2 + B1*B2 + C1*C2);
	 
	 double mian = sqrt(pow(A1,2)+pow(B1,2)+pow(C1,2))*sqrt(pow(A2,2)+pow(B2,2)+pow(C2,2));
	 std::cout << "cosAlfa: " << licz/mian<<"\n";
	 
	 double alfaRad = acos(licz/mian);
	 std::cout << "Alfa [rad]: " << alfaRad<<"\n";
	 
	 double alfaDeg = alfaRad*180/M_PI;
	 std::cout << "Alfa [degree]: " << alfaDeg<<"\n"; 
	 
	 
	 std::cout << "Model inliers: " << inliers->indices.size () << std::endl;
	 pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	 extract.setInputCloud (cloud);
	 extract.setIndices (inliers);
	 extract.setNegative (false);

	 extract.filter (*cloud_inliers);
	 //std::cout << "PointCloud representing the planar component: " << cloud_inliers->points.size () << " data points." << std::endl;

	 // Remove the planar inliers, extract the rest
	 extract.setNegative (true);
	 extract.filter (*cloud_outliers);
	 cloud=cloud_outliers; 
	 
	 if(alfaDeg<prop_alfa_treshold)
	      break;
	 
	 iter++;
}

  PointCloud<PointT>::Ptr object_points  (new PointCloud<PointT>);
  // Step 3c. Project the ground inliers
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
  PointCloud<PointT>::Ptr cloud_projected (new PointCloud<PointT>) ;
  ProjectInliers<PointT>::Ptr proj (new ProjectInliers<PointT>);
  
  proj->setInputCloud(cloud);
  //proj->setIndices(table_inliers);
  proj->setIndices(inliers);
  //proj->setModelType(pcl::SACMODEL_PLANE);
  proj->setModelCoefficients(coefficients);
  proj->filter(*cloud_projected);

  
  // Step 3d. Create a Convex Hull representation of the projected inliers
  PointCloud<PointT>::Ptr ground_hull (new PointCloud<PointT>);
  ConvexHull<PointT> chull;
  chull.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(*cloud_projected));
  chull.reconstruct(*ground_hull);


  std::cout<< "Convex hull has: "<< (int) ground_hull->points.size () <<" data points.\n";
  
  //hull_pub.publish(ground_hull);

  // Step 3e. Extract only those outliers that lie above the ground plane's convex hull
  
  
  pcl::PointIndices object_indices;
  ExtractPolygonalPrismData<PointT> prism;
  prism.setInputCloud(boost::make_shared<CloudT>(*cloud));
  prism.setInputPlanarHull(boost::make_shared<CloudT>(*ground_hull));
  prism.setHeightLimits(0, 0.5);
  prism.segment(object_indices);
  std::cout<<"Extraction\n";
  ExtractIndices<PointT> object_extractor;
  object_extractor.setInputCloud(boost::make_shared<CloudT>(*cloud));
  object_extractor.setIndices(boost::make_shared<PointIndices>(object_indices));
  object_extractor.filter(*object_points);

  std::cout<<"Write data\n";
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrObjPoints (new pcl::PointCloud<pcl::PointXYZRGB> (object_points));
  //out_outliers.write(cloud_outliers);
  out_outliers.write(cloud_projected);
  out_inliers.write(cloud_inliers);
  out_object_points.write(object_points);

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers
// pcl::PointCloud<pcl::PointXYZRGB>  object_points;

}



} //: namespace RANSACPlaneRemover
} //: namespace Processors
