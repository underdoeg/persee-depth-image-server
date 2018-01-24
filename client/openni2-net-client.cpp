//
// Created by phwhitfield on 1/24/18.
//

#include "openni2-net-client.h"

boost::asio::io_service OpenNI2NetClient::ioService;

OpenNI2NetClient::OpenNI2NetClient(unsigned int p):port(p) {
	start();
}

OpenNI2NetClient::~OpenNI2NetClient() {
	bKeepRunning = false;
	if(thread.joinable()) thread.join();
}

void OpenNI2NetClient::setCallback(const OpenNI2NetClient::CallbackCv &callback) {
	callbackCv = callback;
}

void OpenNI2NetClient::start() {
	thread = std::thread([&]{

		using boost::asio::ip::tcp;

		bKeepRunning = true;
		tcp::socket socket(ioService);
		tcp::acceptor acceptor(ioService, tcp::endpoint(tcp::v4(), port));
		acceptor.accept(socket);

		size_t bufferSize = 0;

		int counter = 0;

		std::vector<uint8_t> buffer;

		while(bKeepRunning){

			OpenNI2NetHeader header;

			boost::system::error_code error;

			auto amt = socket.read_some(boost::asio::buffer(&header, sizeof(OpenNI2NetHeader)), error);

			if(amt == 0) continue;

			buffer.clear();
			buffer.reserve(header.size);

			while(bKeepRunning && buffer.size() < header.size){
				std::array<unsigned char, 2048> data;
				size_t toRead = data.size();
				if(toRead > header.size - buffer.size()){
					toRead = header.size - buffer.size();
				}
				amt = socket.read_some(boost::asio::buffer(data, toRead), error);

				buffer.insert(buffer.end(), data.begin(), data.begin()+amt);
			}

			if(!bKeepRunning) break;

			if(buffer.size() != header.size){
				LOGE << "Wrong buffer size received  " << std::endl;
				continue;
			}

			cv::Mat mat(header.height, header.width, CV_16UC1);

			if(header.jpeg == 0) {
				memcpy(mat.data, buffer.data(), buffer.size());
			}else{
				cv::imdecode(buffer, cv::IMREAD_ANYDEPTH, &mat);
			}

			if(callbackCv) callbackCv(mat);
		}

		acceptor.close();
		socket.close();
	});
}
