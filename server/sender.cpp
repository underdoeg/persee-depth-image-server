//
// Created by phwhitfield on 1/24/18.
//

#include <boost/cast.hpp>

#include "sender.h"

Sender::Sender(const std::string &h, unsigned p) :
		socket(ioService), host(h), port(p) {
	bConnected = false;
	bKeepRunning = true;

	LOGI << "Starting streaming to " << host << ":" << port;

	thread = std::thread([&] {
		unsigned connectCounter = 0;

		while (bKeepRunning) {

			if (!bConnected) {
				if (!connect()) {
					if(connectCounter % 50 == 0) {
						LOGW << "Cannot connect to server " << host << ":" << port << " -> will keep retrying";
					}
					connectCounter++;
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
			}


			try {
				// wait for data
				std::unique_lock<std::mutex> l(cvMtx);
				if (cv.wait_for(l, std::chrono::seconds(1)) == std::cv_status::timeout) {
					continue;
				}

				// data size
				OpenNI2NetHeader header = {
						boost::numeric_cast<OpenNI2SizeType>(dataToSend.total() * dataToSend.elemSize()),
						boost::numeric_cast<OpenNI2SizeType>(dataToSend.cols),
						boost::numeric_cast<OpenNI2SizeType>(dataToSend.rows)
				};
				socket.send(boost::asio::buffer(&header, sizeof(OpenNI2NetHeader)));

				// send actual data
				socket.send(boost::asio::buffer(dataToSend.data, header.size));
				//LOGI << "Sending " << header.size / 1000 << "kb";
			} catch (std::exception &e) {
				LOGW << "Client disconnected: " << e.what() << std::endl;
				bConnected = false;
			}
		}
	});
}

Sender::~Sender() {
	bKeepRunning = false;
	if (thread.joinable()) {
		thread.join();
	}
}

bool Sender::connect() {
	try {
		tcp::resolver resolver(ioService);

		tcp::resolver::query query(tcp::v4(), host, std::to_string(port));
		tcp::resolver::iterator iterator = resolver.resolve(query);
		boost::asio::connect(socket, iterator);
		socket.set_option(tcp::no_delay(true));

		bConnected = true;
	} catch (std::exception &e) {
		//LOGW << "Exception: " << e.what() << std::endl;
		bConnected = false;
	}

	return bConnected;
}


void Sender::send(const cv::Mat &mat) {
	{
		std::lock_guard<std::mutex> lk(cvMtx);
		mat.copyTo(dataToSend);
	}
	cv.notify_all();
}

bool Sender::isConnected() {
	return bConnected;
}

