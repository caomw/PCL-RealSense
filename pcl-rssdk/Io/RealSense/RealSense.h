#ifndef _REALSENSE_H_
#define _REALSENSE_H_

#include "pxcsensemanager.h"
#include <iostream>
#include <memory>
#include <string>
#include <functional>
#include <vector>

namespace hv
{
	namespace io
	{
		class RealSense
		{
		public:
			RealSense();
			~RealSense();

			void StartStream(bool blocking);

			std::function<void(std::vector<PXCPoint3DF32>, PXCImage::ImageData,int,int)> NewVerticesCallback;

		private:
			PXCSenseManager *sm;		// Sense Manager
			PXCCaptureManager *cm;		// Capture Manager
			PXCCapture::Device *device;	// Device
			PXCCapture *capture;		// Capture
			PXCProjection *projection;	// Projection Helper
			void* eventHandler;			// Callback event handler

			void Process(pxcUID id, PXCCapture::Sample* sample, pxcStatus* status);


		};

		template <typename T>
		inline T* Check(T* a, std::string str)
		{
			if (!a) {
				throw new std::exception((str + std::string(" failed ")).c_str());
			}
			return a;
		};

		template <typename T>
		inline T Check(T status, std::string str)
		{
			if (typeid(T) == typeid(pxcStatus))
			{
				if (status != pxcStatus::PXC_STATUS_NO_ERROR)
				{
					throw new std::exception((str + std::string(" failed ") + std::to_string(status)).c_str());
				}
			}
			return status;
		};

	}
}


#endif // !_REALSENSE_H

