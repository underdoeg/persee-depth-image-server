#pragma once

#include <gstreamermm.h>

#include "sender.h"

class SenderGstreamer: public Sender{

	Glib::RefPtr<Gst::Pipeline> pipeline;
	Glib::RefPtr<Gst::AppSrc> appSrc;

	std::string host;
	unsigned port;

	cv::VideoWriter writer;

public:
	SenderGstreamer(const std::string &host, unsigned int port);

	void send(const cv::Mat& mat) override;
	void setFov(OpenNI2SizeType fov) override;
};