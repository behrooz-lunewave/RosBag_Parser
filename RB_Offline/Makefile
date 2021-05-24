	
All: LW_Rosbag1 LW_Rosbag2

All_noe:  LW_Rosbag1_noe  LW_Rosbag2_noe

LW_Rosbag1:  LW_Rosbag1.cpp
	g++ -std=c++17 -pthread -w LW_Rosbag.1cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lstdc++fs -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o LW_Rosbag1
	
LW_Rosbag1_noe:  LW_Rosbag1.cpp
	g++ -std=c++17 -pthread -w LW_Rosbag1.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lstdc++fs -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o LW_Rosbag1_noe
	
LW_Rosbag2:  LW_Rosbag2.cpp
	g++ -std=c++17 -pthread -w LW_Rosbag2.cpp  -I/usr/include/opencv4 -I/opt/ros/melodic/include \
	`pkg-config --cflags --libs opencv` \
	-I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/boost $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lstdc++fs -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/melodic/lib \
	-o LW_Rosbag2
	
LW_Rosbag2_noe:  LW_Rosbag2.cpp
	g++ -std=c++17 -pthread -w LW_Rosbag2.cpp  -I/usr/include/opencv4 -I/opt/ros/noetic/include \
	`pkg-config --cflags --libs opencv4` \
	-I/usr/include/eigen3 -I/usr/include/boost     $(pkg-config --cflags eigen3) -lboost_system -lboost_filesystem \
	-lcv_bridge -limage_transport  -lcpp_common -lstdc++fs -lroscpp -lrosconsole -lroscpp_serialization -lrostime  -lrosbag  -lrosbag_storage -L/opt/ros/noetic/lib \
	-o LW_Rosbag2_noe