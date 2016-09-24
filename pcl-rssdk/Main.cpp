
#include "Helper\PointHelper.h"
#include "Core\PCL\Pcl.h"
#include "Io\RealSense\RealSense.h"

using namespace hv::core;
using namespace hv::io;
using namespace hv::helper;

RealSense* sense;
Pcl<PointXYZRGBA>* pointcloud;

void main()
{
	try 
	{
		sense = new RealSense();
		pointcloud = new Pcl<PointXYZRGBA>();

		sense->NewVerticesCallback = [](std::vector<PXCPoint3DF32> input, PXCImage::ImageData data, int width, int height) {

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
			cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width, height));
			cloud->is_dense = false;

			convertPoint(input, data, width, height, cloud);

			pointcloud->AddPoints(cloud);
		};

		sense->StartStream(false);
		pointcloud->Run();
	}
	catch (std::exception ex)
	{
		std::cout << ex.what();
	}
	
	//system("PAUSE");
}
