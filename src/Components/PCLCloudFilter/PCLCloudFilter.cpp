/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include <memory>
#include <string>

#include "PCLCloudFilter.hpp"
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


namespace Processors {
namespace PCLCloudFilter {

PCLCloudFilter::PCLCloudFilter(const std::string & name) :
		Base::Component(name),
		prop_ZAxisMin_treshold("ransacFilter.ZAxisMin_treshold", 0.5),
		prop_ZAxisMax_treshold("ransacFilter.ZAxisMax_treshold", 1.0)
{
		prop_ZAxisMin_treshold.addConstraint("0.0");
		prop_ZAxisMin_treshold.addConstraint("2.0");
		prop_ZAxisMax_treshold.addConstraint("0.0");
		prop_ZAxisMax_treshold.addConstraint("2.0");
		registerProperty(prop_ZAxisMin_treshold);
		registerProperty(prop_ZAxisMax_treshold);
} 

PCLCloudFilter::~PCLCloudFilter() {
  
}

void PCLCloudFilter::prepareInterface() {
  
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_pcl", &in_cloud_pcl);
	registerStream("out_filtered_cloud_pcl", &out_filtered_cloud_pcl);
	
	// Register handlers
	h_filter.setup(boost::bind(&PCLCloudFilter::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_cloud_pcl);

}

bool PCLCloudFilter::onInit() {

	return true;
}

bool PCLCloudFilter::onFinish() {
	return true;
}

bool PCLCloudFilter::onStop() {
	return true;
}

bool PCLCloudFilter::onStart() {
	return true;
}

void PCLCloudFilter::filter() {
  
  // create two clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_pcl.read();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  /*pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.5, 0.6);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.5, 0.6);*/
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (prop_ZAxisMin_treshold, prop_ZAxisMax_treshold);
  //pass.setFilterFieldName ("rgb");
  //pass.setFilterLimits (0.5, 0.6);

  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // test
  /*
  std::cout << "Cloud after filtering: \n" ;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
    if(i%1000==0)
    std::cout << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z <<  "\n";
			
   }
   */
   std::cout <<"Koniec fitrlowania!!!\n";
   std::cout <<"Pozostalo po fitrowaniu: " << cloud_filtered->points.size ()<<" punktow\n";

   // write to output
   out_filtered_cloud_pcl.write(cloud_filtered);
}



} //: namespace PCLCloudFilter
} //: namespace Processors
