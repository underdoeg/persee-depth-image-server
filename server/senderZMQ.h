#pragma once

#include <zmq.hpp>
#include <thread>
#include <condition_variable>
#include <atomic>

#include "sender.h"


class SenderZMQ: public Sender{
	zmq::context_t ctx;
	zmq::socket_t publisher;

	OpenNI2SizeType fovx, fovy;

public:
	SenderZMQ(int port=OpenNI2ServerDefaultPort);

	void send(const cv::Mat& mat) override;
	void setFov(OpenNI2SizeType fx, OpenNI2SizeType fy) override;
};