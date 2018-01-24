#pragma once

#include <thread>
#include <functional>

#include <OpenNI.h>

#include <openni2-net-common.h>

class Grabber: public openni::VideoStream::NewFrameListener{

	using Callback = std::function<void(const openni::DepthPixel*,int,int)>;

	openni::VideoStream stream;
	openni::Device device;

	Callback callback;

public:
	Grabber();
	~Grabber();

	int getWidth();
	int getHeight();

	void setCallback(const Callback& c);
	void start();

private:
	void onNewFrame(openni::VideoStream& in) override;
};
