#include <iostream>
#include <mutex>
#include <atomic>

#include <opencv/cv.hpp>

#include <openni2-net-common.h>

#include "grabber.h"

#include "sender.h"

int main(int argc, char** argv) {

	std::string host = "127.0.0.1";
	auto port = OpenNI2ServerDefaultPort;
	bool compressed = true;
	int compressionQuality = 90;

	LOGI << "optional usage ./openni2-net-stream-server [host] [port] [use compression (0 or 1, on by default)]";

	if(argc > 1){
		host = argv[1];
		if(argc > 2)
			port = std::atoi(argv[2]);
		if(argc > 3){
			if(argv[3][0] != '1') compressed = false;
		}
		if(argc > 4)
			compressionQuality = std::atoi(argv[4]);
	}

	Grabber grabber;
	Sender sender(host, port);
	sender.setCompressed(compressed);
	sender.setCompressionQuality(compressionQuality);

	const std::string windowName = "OpenNI2 Net Server";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

	cv::Mat matThread, mat;
	std::mutex mtx;
	std::atomic_bool bNewMat;
	bNewMat = false;

	grabber.setCallback([&](const openni::DepthPixel* pixels, int w, int h){

		cv::Mat depthMat(h, w, CV_16UC1);
		auto size = depthMat.step[0] * depthMat.rows;
		memcpy(depthMat.data, pixels, size);

		mtx.lock();
		depthMat.copyTo(matThread);
		mtx.unlock();

		bNewMat = true;

		sender.send(depthMat);
	});

	grabber.start();

	while(cv::waitKey(10) != 27){
		if(bNewMat) {
			mtx.lock();
			matThread.convertTo(mat, CV_8U, 255.f / 3000.f);
			mtx.unlock();
			cv::imshow(windowName, mat);
			bNewMat = false;
		}
	}

	return EXIT_SUCCESS;
}