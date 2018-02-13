#pragma once

#include <functional>
#include <thread>
#include <atomic>

#include <boost/asio.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv/cv.hpp>

#include <openni2-net-common.h>

class OpenNI2NetClient{
public:
	using CloudPoint = pcl::PointXYZ;
	using Cloud = pcl::PointCloud<CloudPoint>;

	using CallbackCv = std::function<void(const cv::Mat&)>;
	using CallbackPcl = std::function<void(const Cloud::ConstPtr&)>;

	//static boost::asio::io_service ioService;
	//static bool ioServiceRunning;

private:
	std::thread thread;
	std::atomic_bool bKeepRunning;
	unsigned port;
	std::shared_ptr<boost::asio::io_service> ioService;
	std::shared_ptr<boost::asio::ip::tcp::socket> socket;
	std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;

	CallbackCv callbackCv;
	CallbackPcl callbackPcl;
	Cloud::Ptr cloud;

	std::atomic_int64_t fps;

	void start();

public:
	explicit OpenNI2NetClient(unsigned port = OpenNI2ServerDefaultPort);
	~OpenNI2NetClient();

	void setCallbackCv(const CallbackCv &callback);
	void setCallbackPcl(const CallbackPcl &callback);

	void stop();

	float getFps();
};