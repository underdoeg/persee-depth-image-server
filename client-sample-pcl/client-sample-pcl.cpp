#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <openni2-net-client.h>

int main(int argc, char** argv){

	unsigned port = OpenNI2ServerDefaultPort;

	pcl::console::parse_argument(argc, argv, "-port", port);

	OpenNI2NetClient client(port);

	pcl::visualization::PCLVisualizer viewer("OpenNI2 Stream client PCL");

	viewer.setSize(1920, 1080);
	viewer.setCameraPosition(0, 0, -1, 0, 0, 1, 0, -1, 0);

	auto tmpCloud = OpenNI2NetClient::Cloud::Ptr(new OpenNI2NetClient::Cloud());

	viewer.addPointCloud<OpenNI2NetClient::CloudPoint>(tmpCloud, "cloud", 0);

	std::timed_mutex mtx;
	std::atomic_bool bNewMat;
	bNewMat = false;

	client.setCallbackPcl([&](const OpenNI2NetClient::Cloud::ConstPtr& in) {
		using namespace std::chrono_literals;
		if (mtx.try_lock_for(20ms)) {
			viewer.updatePointCloud<OpenNI2NetClient::CloudPoint>(in, "cloud");
			mtx.unlock();
		}
	});

	while (!viewer.wasStopped()) {
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(10ms);

		std::unique_lock<std::timed_mutex> lock(mtx);
		viewer.spinOnce();
	}

	client.stop();

	viewer.close();

	return EXIT_SUCCESS;
}