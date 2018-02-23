//
// Created by phwhitfield on 1/19/18.
//

#include <boost/cast.hpp>

#include <OniCTypes.h>
#include "grabber.h"
#include <cmath>

Grabber::Grabber() {
}

Grabber::~Grabber() {
	stream.stop();
	device.close();
	openni::OpenNI::shutdown();
}

void Grabber::setCallback(const Grabber::Callback &c) {
	callback = c;
}

void Grabber::start() {
	// create cloud

	// find openni device
	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK) {
		printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
		return;
	}


	rc = device.open(openni::ANY_DEVICE);
	if (rc != openni::STATUS_OK) {
		printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
		return;
	}

	if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL) {
		rc = stream.create(device, openni::SENSOR_DEPTH);
		if (rc != openni::STATUS_OK) {
			printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
			return;
		}
	}

	rc = stream.start();
	if (rc != openni::STATUS_OK){
		printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
		return;
	}

	stream.addNewFrameListener(this);

	// start openni device
	stream.start();
}



void Grabber::onNewFrame(openni::VideoStream &in) {
	if(!callback) return;

	float frameWidth = stream.getVideoMode().getResolutionX();
	float frameHeight = stream.getVideoMode().getResolutionY();
	float hFov = stream.getHorizontalFieldOfView();
	float wFov = stream.getVerticalFieldOfView();
	fovx = boost::numeric_cast<OpenNI2SizeType>(wFov*OpenNI2FloatConversion); //frameWidth / (2.0f * std::tan(wFov / 2.0f)) * OpenNI2FloatConversion);
	fovy = boost::numeric_cast<OpenNI2SizeType>(hFov*OpenNI2FloatConversion); //frameHeight / (2.0f * std::tan(wFov / 2.0f)) * OpenNI2FloatConversion);

	openni::VideoFrameRef ref;
	in.readFrame(&ref);

	const int width = ref.getWidth();
	const int height = ref.getHeight();
	auto pixels = static_cast<const openni::DepthPixel*>(ref.getData());

	callback(pixels, width, height);

	/*
	if(callbackPcl){
		if(!cloud) cloud = Grabber::Cloud::Ptr(new Grabber::Cloud());


		cloud->resize(width * height);

		for (int v = 0; v < height; ++v) {
			for (int u = 0; u < width; ++u) {
				int index = v*width+u;
				auto& a = cloud->at(index);
				openni::CoordinateConverter::convertDepthToWorld(in, u, v, pixels[index], &a.x, &a.y, &a.z);
			}
		}

		callbackPcl(cloud);
	}
	*/
}

int Grabber::getWidth() {
	return stream.getVideoMode().getResolutionX();
}

int Grabber::getHeight() {
	return stream.getVideoMode().getResolutionY();
}

OpenNI2SizeType Grabber::getFovX() {
	return fovx;
}

OpenNI2SizeType Grabber::getFovY() {
	return fovy;
}

/*
void Grabber::setCallbackPcl(const Grabber::CallbackPcl &c) {
	callbackPcl = c;
}
*/
