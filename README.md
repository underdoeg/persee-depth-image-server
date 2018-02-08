# OpenNi2 depth image streaming server and client

This example project helps with streaming the depth data of any OpenNI device. Though it was specifically developed for the ORbbec Persee and contains the proper OpenNI libs.

The server is a standalone application and requires opencv3. Once built it can stream the data to any ip endpoint.

The client library can be embedded in your application or two example applications visualize the point cloud either as a depth image with opencv or a pcl point cloud.
