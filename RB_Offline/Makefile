	
LW_Rosbag:  LW_Rosbag.cpp
	g++ -std=c++11 -pthread -w LW_Rosbag.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o LW_Rosbag
	

	
LW_Rosbag_noe:  LW_Rosbag.cpp
	g++ -std=c++11 -pthread -w LW_Rosbag.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o LW_Rosbag
	
