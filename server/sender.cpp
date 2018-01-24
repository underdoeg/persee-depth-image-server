//
// Created by phwhitfield on 1/24/18.
//

#include <boost/cast.hpp>

#include "sender.h"

Sender::Sender(const std::string &h, unsigned p) :
		socket(ioService), host(h), port(p) {
	bConnected = false;
	bKeepRunning = true;

	thread = std::thread([&] {
		while (bKeepRunning) {

			if (!bConnected) {
				if (!connect()) {
					LOGE << "Cannot connect to server " << host << ":" << port << " -> will not send data";
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
			}

			// wait for data
			std::unique_lock<std::mutex> l(cvMtx);
			if (cv.wait_for(l, std::chrono::seconds(1)) == std::cv_status::timeout) {
				LOGI << "NOPE";
				continue;
			}

			auto size = boost::numeric_cast<OpenNI2SizeType>(dataToSend.size());
			socket.send(boost::asio::buffer(&size, sizeof(OpenNI2SizeType)));
			socket.send(boost::asio::buffer(dataToSend.data(), dataToSend.size()));
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
		LOGE << "Exception: " << e.what() << std::endl;
		bConnected = false;
	}

	return bConnected;
}


void Sender::send(size_t size, const uint8_t *data) {
	{
		std::lock_guard<std::mutex> lk(cvMtx);
		dataToSend.resize(size);
		memcpy(dataToSend.data(), data, size);
	}
	cv.notify_all();
}

bool Sender::isConnected() {
	return bConnected;
}

