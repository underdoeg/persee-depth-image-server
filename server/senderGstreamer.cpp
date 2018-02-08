//
// Created by phwhitfield on 2/7/18.
//

#include "senderGstreamer.h"

SenderGstreamer::SenderGstreamer(const std::string &h, unsigned int p) : host(h), port(p) {
	Gst::init();

	const std::string launchStr = "appsrc name=src ! video/x-raw,format=GRAY16_LE,width=640,height=480,framerate=30 ! autovideosink"; //openjpegenc ! rtpj2kpay ! rtpj2kdepay ! avdec_jpeg2000 ! videoconvert ! autovideosink"; //video/x-raw,format=GRAY16_LE,width=640,height=480 !

	pipeline = Glib::RefPtr<Gst::Pipeline>::cast_dynamic(Gst::Parse::launch(launchStr));

	if (!pipeline) {
		LOGF << "Could not create pipeline";
		return;
	}

	appSrc = Glib::RefPtr<Gst::AppSrc>::cast_dynamic(pipeline->get_child("src"));

	if(!appSrc){
		LOGE << "Could not get app src";
	}

	appSrc->set_caps(Gst::Caps::create_simple(
			"video/x-raw",
			"format", "GRAY16_LE",
			"width", 640,
			"height", 480,
			"framerate", 30
	));

	appSrc->signal_need_data().connect([&](auto size){
		LOGI << size;
	});

	pipeline->set_state(Gst::State::STATE_PLAYING);
}

void SenderGstreamer::send(const cv::Mat &mat) {
	size_t size = mat.rows * mat.cols * sizeof(uint16_t);
	LOGI << size;
	auto buffer = Gst::Buffer::create(size);

	Gst::MapInfo mapInfo;
	buffer->map(mapInfo, Gst::MAP_WRITE);
	mempcpy(mapInfo.get_data(), mat.data, size);
	buffer->unmap(mapInfo);

	appSrc->push_buffer(buffer);
}

void SenderGstreamer::setFov(OpenNI2SizeType fov) {

}

