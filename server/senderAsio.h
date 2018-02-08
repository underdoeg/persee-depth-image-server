#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <boost/asio.hpp>

#include <opencv/cv.hpp>

#include <openni2-net-common.h>

#include "sender.h"

using boost::asio::ip::tcp;

class SenderAsio: public Sender{

	std::condition_variable cv;
	std::mutex cvMtx;
	cv::Mat matToSend;
	std::thread thread;
	std::atomic_bool bKeepRunning;

	boost::asio::io_service ioService;
	tcp::socket socket;
	std::atomic_bool bConnected;
	std::atomic_bool bUseCompression;
	std::atomic_int compressionQuality;

	OpenNI2SizeType fov;

	std::string host;
	unsigned port;

	bool connect();

public:
	explicit SenderAsio(const std::string &h, unsigned p);

	~SenderAsio();

	void send(const cv::Mat& mat) override;

	bool isConnected();
	void setCompressed(bool state=true);
	void setCompressionQuality(int quality);
	void setFov(OpenNI2SizeType fov) override;
};