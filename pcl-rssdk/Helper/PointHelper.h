#ifndef _POINTHELPER_H_
#define _POINTHELPER_H_

#include <vector>
#include <pxcsensemanager.h>
#include <pcl\visualization\cloud_viewer.h>

using namespace std;
using namespace pcl;

namespace hv
{
	namespace helper
	{

		template<typename T> inline void
			convertPoint(vector<PXCPoint3DF32> xyzsrc, PXCImage::ImageData data, int width, int height, T& tgt)
		{

			float nan = numeric_limits<float>::quiet_NaN();
			uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);

#pragma omp parallel num_threads(8)
			for (int i = 0; i < height; i++)
			{
				pcl::PointXYZRGBA* cloud_row = &tgt->points[i * width];
				uint32_t* color_row = &d[i * data.pitches[0] / sizeof(uint32_t)];
				for (int j = 0; j < width; j++)
				{
					PXCPoint3DF32 point = xyzsrc.at(i* width + j );
					if (point.z == 0)
					{
						// TODO: instead of adding and removing, find more efficient way to handle this
						cloud_row[j].x = cloud_row[j].y = cloud_row[j].z = nan;
						continue;
					}
					cloud_row[j].x = point.x / 1000; //convert to meters
					cloud_row[j].y = point.y / 1000;
					cloud_row[j].z = point.z / 1000;

					if (color_row[j] == 0)
					{
						continue;
					}
					
					memcpy(&cloud_row[j].rgb, &color_row[j], sizeof(uint32_t));
				}					
			}
			// remove nan elements
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tgt, *tgt, index);
		};
	};
}
#endif

