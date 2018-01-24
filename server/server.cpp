#include <iostream>
#include <mutex>
#include <atomic>

#include <opencv/cv.hpp>

#include <common.h>

#include "grabber.h"

#include "sender.h"

int main(int argc, char** argv) {

	LOGI << "START OPENNI2 SERVER";

	Grabber grabber;
	Sender sender("127.0.0.1", OpenNI2ServerDefaultPort);

	cv::namedWindow("win", cv::WINDOW_NORMAL);

	cv::Mat matThread, mat;
	std::mutex mtx;
	std::atomic_bool bNewMat;
	bNewMat = false;

	grabber.setCallback([&](const openni::DepthPixel* pixels, int w, int h){

//		std::vector<int> params;
//		params.push_back(CV_IMWRITE_JPEG_QUALITY);
//		params.push_back(80);


		cv::Mat depthMat(h, w, CV_16UC1);
		auto size = depthMat.step[0] * depthMat.rows;
		memcpy(depthMat.data, pixels, size);

		mtx.lock();
		matThread = depthMat.clone();
		mtx.unlock();

		bNewMat = true;

		sender.send(size, reinterpret_cast<const uint8_t*>(pixels));
	});

	grabber.start();

	while(cv::waitKey(10) != 27){
		if(bNewMat) {
			mtx.lock();
			matThread.convertTo(mat, CV_8U, 255.f / 3000.f);
			mtx.unlock();
			cv::imshow("win", mat);
			bNewMat = false;
		}
	}

	return EXIT_SUCCESS;
}