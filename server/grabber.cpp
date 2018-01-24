//
// Created by phwhitfield on 1/19/18.
//

#include <OniCTypes.h>
#include "grabber.h"

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

	LOGI << stream.getVideoMode().getResolutionX();

	// start openni device
	stream.start();
}



void Grabber::onNewFrame(openni::VideoStream &in) {
	if(!callback) return;

	openni::VideoFrameRef ref;
	in.readFrame(&ref);

	const int width = ref.getWidth();
	const int height = ref.getHeight();
	auto pixels = static_cast<const openni::DepthPixel*>(ref.getData());

	callback(pixels, width, height);
}

int Grabber::getWidth() {
	return stream.getVideoMode().getResolutionX();
}

int Grabber::getHeight() {
	return stream.getVideoMode().getResolutionY();
}
