#pragma once

#include <functional>
#include <thread>

#include <boost/asio.hpp>

#include <opencv/cv.hpp>

#include <openni2-net-common.h>

class OpenNI2NetClient{
public:
	using CallbackCv = std::function<void(const cv::Mat&)>;

	static boost::asio::io_service ioService;

private:
	std::thread thread;
	std::atomic_bool bKeepRunning;
	unsigned port;

	CallbackCv callbackCv;

	void start();

public:
	explicit OpenNI2NetClient(unsigned port = OpenNI2ServerDefaultPort);
	~OpenNI2NetClient();

	void setCallback(const CallbackCv& callback);
};