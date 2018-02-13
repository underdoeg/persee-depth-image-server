//
// Created by phwhitfield on 1/24/18.
//

#include <chrono>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

//#include <pcl/compression/organized_pointcloud_conversion.h>

#include "openni2-net-client.h"

boost::asio::io_service OpenNI2NetClient::ioService;

OpenNI2NetClient::OpenNI2NetClient(unsigned int p):port(p) {
	cloud = Cloud::Ptr(new Cloud());
	start();
}

OpenNI2NetClient::~OpenNI2NetClient() {
	stop();
}


void OpenNI2NetClient::stop() {
	LOGI << "Stop Openni Listener on port " << port;
	bKeepRunning = false;
	if(thread.joinable()) thread.join();
}


void OpenNI2NetClient::setCallbackCv(const OpenNI2NetClient::CallbackCv &callback) {
	callbackCv = callback;
}

void OpenNI2NetClient::setCallbackPcl(const OpenNI2NetClient::CallbackPcl &callback) {
	callbackPcl = callback;
}



void OpenNI2NetClient::start() {
	LOGI << "Listening for openni data on port " << port;

	thread = std::thread([&]{

		using boost::asio::ip::tcp;

		bKeepRunning = true;
		tcp::socket socket(ioService);
		tcp::acceptor acceptor(ioService, tcp::endpoint(tcp::v4(), port));
		acceptor.accept(socket);

		size_t bufferSize = 0;

		int counter = 0;

		std::vector<uint8_t> buffer;
		OpenNI2NetHeader header;
		boost::system::error_code error;

		while(bKeepRunning){

			auto start = std::chrono::high_resolution_clock::now();

			auto amt = socket.read_some(boost::asio::buffer(&header, sizeof(OpenNI2NetHeader)), error);

			if(amt == 0){continue;};

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
			if(callbackPcl){

				const uint16_t* depthMap = reinterpret_cast<const uint16_t*>(mat.data);

				cloud->width = header.width;
				cloud->height = header.height;
				cloud->is_dense = false;
				cloud->points.resize(cloud->height * cloud->width);


				float fx = header.fovx / float(OpenNI2FloatConversion); // Horizontal focal length
				float fy = header.fovy / float(OpenNI2FloatConversion); // Vertcal focal length
				float cx = ((float)cloud->width - 1.f) / 2.f;  // Center x
				float cy = ((float)cloud->height - 1.f) / 2.f; // Center y

				float fx_inv = 1.0f / fx;
				float fy_inv = 1.0f / fy;

				int depth_idx = 0;
				float bad_point = std::numeric_limits<float>::quiet_NaN ();

				for (int v = 0; v < cloud->height; ++v)
				{
					for (int u = 0; u < cloud->width; ++u, ++depth_idx)
					{
						pcl::PointXYZ& pt = cloud->points[depth_idx];
						/// @todo Different values for these cases
						// Check for invalid measurements
						if (depthMap[depth_idx] == 0){
//							depthMap[depth_idx] == depth_image->getNoSampleValue () ||
//							depthMap[depth_idx] == depth_image->getShadowValue ())
							pt.x = pt.y = pt.z = bad_point;
						}
						else{
							pt.z = depthMap[depth_idx] * 0.001f; // millimeters to meters
							pt.x = (static_cast<float> (u) - cx) * pt.z * fx_inv;
							pt.y = (static_cast<float> (v) - cy) * pt.z * fy_inv;
						}
					}
				}
				cloud->sensor_origin_.setZero ();
				cloud->sensor_orientation_.setIdentity ();

				callbackPcl(cloud);
			}

			auto finish = std::chrono::high_resolution_clock::now();

			std::chrono::duration<double> elapsed = finish - start;

			fps = (1.f / elapsed.count()) * 1000;
		}

		acceptor.close();
		socket.close();
	});
}

float OpenNI2NetClient::getFps() {
	return fps / 1000.f;
}


