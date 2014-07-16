/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "PCDWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <Types/PointXYZDescriptor.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace Processors {
namespace PCDWriter {

PCDWriter::PCDWriter(const std::string & name) :
		Base::Component(name),
		filename("filename", std::string(""))   {
			registerProperty(filename);
}

PCDWriter::~PCDWriter() {
}

void PCDWriter::prepareInterface() {
<<<<<<< HEAD
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
    registerStream("in_pcl_xyzsift", &in_pcl_xyzsift);
    registerStream("in_pcl_xyzrgb", &in_pcl_xyzrgb);
	// Register handlers
	h_Write.setup(boost::bind(&PCDWriter::Write, this));
	registerHandler("Write", &h_Write);
    h_Write_xyzsift.setup(boost::bind(&PCDWriter::Write_xyzsift, this));
    registerHandler("Write_xyzsift", &h_Write_xyzsift);
    h_Write_xyzrgb.setup(boost::bind(&PCDWriter::Write_xyzrgb, this));
    registerHandler("Write_xyzrgb", &h_Write_xyzrgb);
    //addDependency("Write", &in_pcl);
=======
	// Register data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
//	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);

	// Register handlers - no dependencies.
	h_Write_xyz.setup(boost::bind(&PCDWriter::Write_xyz, this));
	registerHandler("Write_xyz", &h_Write_xyz);

/*	h_Write_xyzsift.setup(boost::bind(&PCDWriter::Write_xyzsift, this));
    registerHandler("Write_xyzsift", &h_Write_xyzsift);*/

	h_Write_xyzrgb.setup(boost::bind(&PCDWriter::Write_xyzrgb, this));
	registerHandler("Write_xyzrgb", &h_Write_xyzrgb);
>>>>>>> 4505dd193bbdb26b5212796a6f7bdbd608f37403
}

bool PCDWriter::onInit() {

	return true;
}

bool PCDWriter::onFinish() {
	return true;
}

bool PCDWriter::onStop() {
	return true;
}

bool PCDWriter::onStart() {
	return true;
}

void PCDWriter::Write_xyz() {
	LOG(LTRACE) << "PCDWriter::Write_xyzrgb";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::io::savePCDFileASCII (filename, *cloud);
<<<<<<< HEAD
	std::cerr << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;
}

void PCDWriter::Write_xyzsift() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_pcl_xyzsift.read();
    pcl::io::savePCDFileASCII (filename, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;
}

void PCDWriter::Write_xyzrgb() {
	cout<<"Write_xyzrgb()"<<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_pcl_xyzrgb.read();
    pcl::io::savePCDFileASCII (filename, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;
=======
	LOG(LINFO) << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;
	
}

/*void PCDWriter::Write_xyzsift() {
	LOG(LTRACE) << "PCDWriter::Write_xyzrgb";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
	pcl::io::savePCDFileASCII (filename, *cloud);
	LOG(LINFO) << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;

}*/

void PCDWriter::Write_xyzrgb() {
	LOG(LTRACE) << "PCDWriter::Write_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::io::savePCDFileASCII (filename, *cloud);
	LOG(LINFO) << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;

>>>>>>> 4505dd193bbdb26b5212796a6f7bdbd608f37403
}


} //: namespace PCDWrite
} //: namespace Processors
