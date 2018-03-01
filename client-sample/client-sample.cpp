#include <mutex>

#include <openni2-net-client.h>

static std::mutex mtx;
static cv::Mat matThread, mat, matShort;

void mouseCB(int event, int x, int y, int flags, void *userdata) {
	mtx.lock();
	if (event == cv::EVENT_LBUTTONDOWN)
		LOGI << matShort.at<uint16_t>(y, x);
	mtx.unlock();
}

int main(int argc, char **argv) {

	OpenNI2NetClient client("84.84.84.20", 3344);

	cv::namedWindow("win", cv::WINDOW_AUTOSIZE);


	std::atomic_bool bNewMat;
	bNewMat = false;

	client.setCallbackCv([&](const cv::Mat &in) {
		mtx.lock();
		in.copyTo(matThread);
		mtx.unlock();
		bNewMat = true;
	});

	unsigned fpsCounter = 0;

	cv::setMouseCallback("win", mouseCB, NULL);


	while (cv::waitKey(10) != 27) {
		if (bNewMat) {
			mtx.lock();
			matShort = matThread;
			matThread.convertTo(mat, CV_8U, 255.f / 8000.f);
			mtx.unlock();
			cv::imshow("win", mat);
			bNewMat = false;

			if (fpsCounter % 100 == 0) {
				LOGI << "FPS " << client.getFps();
				LOGI << "FOV " << client.getFovX() << "/" << client.getFovY();
			}
			fpsCounter++;
		}
	}

	return EXIT_SUCCESS;
}
