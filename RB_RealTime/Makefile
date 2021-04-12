	
radar_sub:
	g++ -std=c++11 -pthread -w radar_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o radar_sub
	

	
radar_sub_noe:
	g++ -std=c++11 -pthread -w radar_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o radar_sub
	
camera_sub:
	g++ -std=c++11 -pthread -w camera_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o  camera_sub
	
	
camera_sub_noe:
	g++ -std=c++11 -pthread -w camera_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o camera_sub

lidar_sub:
	g++ -std=c++11 -pthread -w lidar_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o  lidar_sub
	
	
lidar_sub_noe:
	g++ -std=c++11 -pthread -w lidar_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o lidar_sub

all_sub:
	g++ -std=c++11 -pthread -w all_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o  all_sub
	
	
all_sub_noe:
	g++ -std=c++11 -pthread -w all_sub.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o all_sub