#pragma once

#include <opencv/cv.hpp>
#include <openni2-net-common.h>

class Sender{
public:
	virtual void send(const cv::Mat& mat) = 0;
	virtual void setFov(OpenNI2SizeType fov) = 0;
};