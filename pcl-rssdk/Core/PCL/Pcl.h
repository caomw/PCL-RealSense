#ifndef _PCL_H_
#define _PCL_H_

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

namespace hv
{
	namespace core
	{
		template <class PointT>	class Pcl
		{

		public:
			typedef pcl::PointCloud<PointT> PointCloudT;

			/// Constructor
			Pcl()
				:viewer("RealSense Viewer")
			{
				try
				{
					viewer.setCameraFieldOfView(0.785398); // approximately 45 degrees
					viewer.setCameraPosition(0, 0, 1000, 0, 0, 1, 0, 1, 0);

					viewer.setBackgroundColor(0, 0, 0);
					viewer.initCameraParameters();
				}
				catch (std::exception ex)
				{
					cout << ex.what();
				}
			};

			/// Destructor
			~Pcl() {};

			/// Add points to visualizer
			void AddPoints(typename PointCloudT::ConstPtr points)
			{
				if (!viewer.wasStopped())
				{
					boost::mutex::scoped_lock lock(vis_mutex);
					new_cloud_ = points;
				}
			};

			/// Main loop threada
			void Run()
			{
				while (!viewer.wasStopped())
				{
					if (new_cloud_)
					{
						boost::mutex::scoped_lock lock(vis_mutex);
						if (!viewer.updatePointCloud(new_cloud_, "cloud"))
						{
							viewer.addPointCloud(new_cloud_, "cloud");
							viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
						}
						last_cloud_ = new_cloud_;
						new_cloud_.reset();
					}
					viewer.spinOnce(1);
				}
			};

		private:
			typename PointCloudT::ConstPtr new_cloud_;
			typename PointCloudT::ConstPtr last_cloud_;

			pcl::visualization::PCLVisualizer viewer;
			mutable boost::mutex vis_mutex;

		};
	};
};

#endif // !_PCL_H

