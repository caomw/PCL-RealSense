#ifndef _REALSENSEEVENTHANDLER_H_
#define _REALSENSEEVENTHANDLER_H_

#include "pxcsensemanager.h"
#include <functional>

namespace hv
{
	namespace io
	{
		class RealSense;

		class RealSenseEventHandler : public PXCSenseManager::Handler
		{
		public:

			RealSenseEventHandler(std::function<void(pxcUID, PXCCapture::Sample*, pxcStatus*)> func) { Callback = func; };

			virtual pxcStatus PXCAPI OnNewSample(pxcUID id, PXCCapture::Sample *sample)
			{
				pxcStatus status;
				Callback(id, sample, &status);
				return status;
			};

		private:
			std::function<void(pxcUID, PXCCapture::Sample*, pxcStatus*)> Callback;
		};
	}

}

#endif // !_REALSENSEEVENTHANDLER_H_
