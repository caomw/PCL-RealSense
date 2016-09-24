
#include "RealSense.h"
#include "RealSenseEventHandler.h"

using namespace hv::io;
using namespace std::placeholders;

RealSense::RealSense()
{
	eventHandler = new RealSenseEventHandler(std::bind(&RealSense::Process, this, _1, _2, _3));

	PXCVideoModule::DataDesc ddesc = {};
	ddesc.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
	ddesc.streams.color.frameRate.max = 30;
	ddesc.streams.color.frameRate.min = 30;
	ddesc.streams.color.sizeMax.width = 1920;
	ddesc.streams.color.sizeMax.height = 1080;
	ddesc.streams.color.sizeMin.width = 1920;
	ddesc.streams.color.sizeMin.height = 1080;
	ddesc.streams.color.options = PXCCapture::Device::STREAM_OPTION_ANY;

	ddesc.streams.depth.frameRate.max = 30;
	ddesc.streams.depth.frameRate.min = 30;
	//ddesc.streams.depth.sizeMax.width = 640;
	//ddesc.streams.depth.sizeMax.height = 480;
	ddesc.streams.depth.sizeMax.width = 628;
	ddesc.streams.depth.sizeMax.height = 468;
	ddesc.streams.depth.sizeMax.width = 628;
	ddesc.streams.depth.sizeMin.height = 468;
	ddesc.streams.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;

	sm = Check(PXCSenseManager::CreateInstance(), "sm");	// Create sense manager instance	
	Check(sm->EnableStreams(&ddesc),"Enable streams");
	Check(sm->Init((RealSenseEventHandler*)eventHandler), "Init sm");

	cm = Check(sm->QueryCaptureManager(), "cm");			// Query capture manager instance
	capture = Check(cm->QueryCapture(), "capture ");		// Query capture
	device = Check(capture->CreateDevice(0), "device");		// Create device
	projection = device->CreateProjection();
}

RealSense::~RealSense()
{
	if (projection != NULL) { projection->Release(); }
	if (device != NULL) { device->Release(); }
	if (capture != NULL) { capture->Release(); }
	if (cm != NULL) { sm->Release(); }
	if (sm != NULL) { sm->Release(); }

	sm = NULL;
	cm = NULL;
	capture = NULL;
	device = NULL;
	projection = NULL;
}


void RealSense::StartStream(bool blocking)
{
	Check(sm->StreamFrames(blocking), "Start Stream");
}

void RealSense::Process(pxcUID id, PXCCapture::Sample* sample, pxcStatus* status)
{
	if (sample == NULL || sample->depth == NULL || sample->color == NULL)
	{
		*status = pxcStatus::PXC_STATUS_NO_ERROR;
		return;
	}

	// Create info
	PXCImage::ImageInfo dinfo = {}, cinfo = {}, minfo = {};
	PXCImage::ImageData data = {};

	// Query Info
	dinfo = sample->depth->QueryInfo();
	cinfo = sample->color->QueryInfo();

	// Project to depth
	PXCImage* img = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);
	img->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PixelFormat::PIXEL_FORMAT_RGB32, &data);
	minfo = img->QueryInfo();
	
	// Create vertices
	std::vector<PXCPoint3DF32> vertices(dinfo.width * dinfo.height);
	Check(projection->QueryVertices(sample->depth, &vertices[0]), "Depth to 3D Conversion");

	
	// Callback
	if (this->NewVerticesCallback != NULL)
	{
		this->NewVerticesCallback(vertices, data, (int)minfo.width, (int)minfo.height);
	}

	// Clean
	img->ReleaseAccess(&data);
	img->Release();

	*status = pxcStatus::PXC_STATUS_NO_ERROR;
}

