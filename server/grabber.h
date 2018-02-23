#pragma once

#include <thread>
#include <functional>

#include <OpenNI.h>

#include <openni2-net-common.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

class Grabber: public openni::VideoStream::NewFrameListener{
public:

	//using Point = pcl::PointXYZ;
	//using Cloud = pcl::PointCloud<Point>;

private:

	using Callback = std::function<void(const openni::DepthPixel*,int,int)>;
	//using CallbackPcl = std::function<void(const Cloud::ConstPtr)>;

	openni::VideoStream stream;
	openni::Device device;
	//Grabber::Cloud::Ptr cloud;

	Callback callback;
	//CallbackPcl callbackPcl;

	OpenNI2SizeType fovy, fovx;

public:

	Grabber();
	~Grabber();

	int getWidth();
	int getHeight();

	void setCallback(const Callback& c);
	//void setCallbackPcl(const CallbackPcl& c);
	void start();

	OpenNI2SizeType getFovX();
	OpenNI2SizeType getFovY();

private:
	void onNewFrame(openni::VideoStream& in) override;
};
