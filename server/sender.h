#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <boost/asio.hpp>

#include <common.h>

using boost::asio::ip::tcp;

class Sender {

	std::condition_variable cv;
	std::mutex cvMtx;
	std::vector<uint8_t> dataToSend;
	std::thread thread;
	std::atomic_bool bKeepRunning;

	boost::asio::io_service ioService;
	tcp::socket socket;
	std::atomic_bool bConnected;

	std::string host;
	unsigned port;

	bool connect();

public:
	explicit Sender(const std::string &h, unsigned p);

	~Sender();

	void send(size_t size, const uint8_t *data);

	bool isConnected();
};