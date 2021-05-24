
#define CAN

#include <stdio.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <ctime>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <ctime>
#include <sys/stat.h> // stat
#include <unistd.h>
#include <glob.h>
//#include <filesystem>
#include <experimental/filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <boost/foreach.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/imgcodecs.hpp>

#ifdef CAN
#include "ros/ros.h"
#include <can_msgs/Frame.h>
#endif 

#define foreach BOOST_FOREACH
#define PI 3.14159265
#define BUILD_ID  "2028-PC2"

using namespace std;
using namespace cv;
using std::experimental::filesystem::current_path;


char logBuf[128];

union Float {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};

union  Double
{
    double    m_double;
    uint8_t  m_bytes[sizeof(double)];
};

union  Uint16
{
    uint16_t    m_uint16;
    uint8_t  m_bytes[sizeof(uint16_t)];
};

union  Uint32
{
    uint32_t    m_uint32;
    uint8_t  m_bytes[sizeof(uint32_t)];
};


union  Int16
{
    int16_t    m_int16;
    uint8_t  m_bytes[sizeof(int16_t)];
};



void imageCrop(cv::Mat &img, cv::Mat &img_crop, int offset_x, int offset_y, int side)
{

    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;


    if (side)
	{   // crop left side image
        roi.x = offset_x;
        roi.y = offset_y;
    }
    else
	{        // crop right side image
        roi.x = 0;
        roi.y = offset_y;
    }

    roi.width = img.size().width - offset_x;
    roi.height = img.size().height - offset_y;

    img_crop =img(roi);

}

int  Concat(string bag_file, int i)
{

	string camera_dest_1=  bag_file + "Camera_1";
	string camera_dest_2 = bag_file + "Camera_2";
	string camera_dest_3 = bag_file + "Camera_3";


        cv::Mat cam_1 = cv::imread( camera_dest_1 +"/" + std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
        cv::Mat cam_2 = cv::imread( camera_dest_2 +"/"+ std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
        cv::Mat cam_3 = cv::imread( camera_dest_3 +"/"+ std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
        cv::Mat cam_12, cam;

        int image_width = 648;
        int image_height = 496;

        int offset_left_x = 260;
        int offset_left_y = 0;
        int offset_right_x = 235;
        int offset_right_y = 0;

        cv::Mat cam_1_crop;
        cv::Mat cam_3_crop;

		if((cam_1.size().height  == 0) ||  (cam_1.size().width  == 0) ||  (cam_3.size().height  == 0) ||  (cam_3.size().width  == 0))
        {
			return -1;
		}

        imageCrop(cam_1, cam_1_crop, offset_left_x, offset_left_y, 0);    // crop right side image
        imageCrop(cam_3, cam_3_crop, offset_right_x, offset_right_y, 1);  // crop left side image

        cv::hconcat(cam_1_crop, cam_2, cam_12);
		// cv::hconcat(cam_1, cam_2, cam_12);
		  cv::hconcat(cam_12, cam_3_crop, cam);
        //cv::hconcat(cam_12, cam_3, cam);

       // cv::imshow("All cameras", cam);
	   

	   	try
		{
				cv::imwrite(  bag_file + "Camera/concat_" + std::to_string(i) + ".jpg",  cam);
		}
		catch(const cv::Exception& ex)
		{
				cout<< "Exception converting image to PNG  after concatanation  for image  " <<i<<"   "<<endl<< ex.what()<<endl;
		}


    return 0;

}


char radar_1_buffer[4*1024*1024];
char radarBF_buffer[4*1024*1024];
char lidar_buffer[1024*1024*64];

int main (int argc, char **argv)
{
    int min =0;
	int old_min = 0;
	glob_t globbuf;
	ros::Time::init();
	vector<string> fileList;
	vector<string> dest_fileList;
	rosbag::Bag bag;
	

	
	vector<string> ModeSreach = { "64N","64F","128N","128F","256N","256F"};
	
/*	string GPS_dest = bag_if(argc < 2)
	string IMU_dest = bag_w{
	string CAN_dest = bag_	std::cout<< " Error. Please Enter the name the rosbag file. For instance:  $ ./radar_txt  test.bag"<<std::endl;
		exit(1);
	}
	string bag_file 	 = argv[1];*/

	std::cout<<std::endl;
	std::cout << " Lunewave Rosbag file parser " <<  "    -     "  << " Build ID : "<< BUILD_ID << std::endl;
	std::cout << " -------------------------------------------------------------------------------------------- " << std::endl;


    string bag_wildcard = "*.bag";

	 time_t now = time(0);
    tm *ltm = localtime(&now);

	string s;
	 char buffer [80];
    strftime(buffer, 80, "%Y/%m/%d_%H:%M:%S", ltm);
    cout<<  buffer<<endl;

  int err1 = glob((bag_wildcard).c_str(), 0, NULL,  &globbuf);
  if(err1 == 0)
    {
		for (size_t i = 0; i < globbuf.gl_pathc; i++)
		{
		  fileList.push_back(globbuf.gl_pathv[i]);
		}

      globfree(&globbuf);
    }

	if(fileList.size() == 0)
	{
		std::cout<< " No rosbag file is found. exit the program."<<std::endl;
		return 0;
	}


	for(auto it = fileList.begin() ; it != fileList.end(); it++)
	{
	   int k = 0;
	   string detected_mode;
	   string bag_file = (*it);

	   for(int j = 0; j< ModeSreach.size();j++)
	   {
		   if (bag_file.find(ModeSreach[j]) != std::string::npos)
		   {
					detected_mode = ModeSreach[j];
					break;
			}
	   }

     
	    cout<<bag_file<<endl;
	   bag.open(  bag_file, rosbag::bagmode::Read);
      std::vector<std::string> bag_topics;
  

	bool radar_1_exist = false;
    bool radar_2_exist = false;
	bool radar_bf_exist = false;
    bool lidar_exist = false;
	bool camera_1_exist = false;
    bool camera_2_exist  = false;
	bool camera_3_exist  =  false;
	bool camera_exist  =  false;
	bool  GPS_exist = false;
	bool  IMU_exist = false;
	bool  CAN_exist = false;
       bag_topics.push_back(std::string("/radar"));
	   bag_topics.push_back(std::string("/radar_bf"));
       bag_topics.push_back(std::string("/pandar40p"));
       bag_topics.push_back(std::string("/camera/cam_1"));
	   bag_topics.push_back(std::string("/camera/cam_2"));
	   bag_topics.push_back(std::string("/camera/cam_3"));
	   bag_topics.push_back(std::string("/IMU"));
	   bag_topics.push_back(std::string("/GPS"));
       bag_topics.push_back(std::string("/CAN_msg"));
   
	rosbag::View view(bag, rosbag::TopicQuery(bag_topics));

   

		//ros::Rate loop_rate(20);

		//int i =0;

		string bag_wthExt = bag_file.substr(0, bag_file.size() - 4);
		
	

		mkdir(bag_wthExt.c_str(), 0777);

		string bag_wthExtDir = bag_wthExt + "/" ;

		string radar_1_dest = bag_wthExtDir+ "Radar";
		string radar_2_dest = bag_wthExtDir+ "Radar_2";
		string radar_bf_dest = bag_wthExtDir+ "Radar_bf";
		string lidar_dest = bag_wthExtDir + "Lidar";
		string camera_dest_1= bag_wthExtDir+ "Camera_1";
		string camera_dest_2 = bag_wthExtDir+ "Camera_2";
		string camera_dest_3 = bag_wthExtDir + "Camera_3";
		string camera_dest = bag_wthExtDir + "Camera";
		string GPS_dest = bag_wthExtDir + "GPS";
		string IMU_dest = bag_wthExtDir + "IMU";
		string CAN_dest = bag_wthExtDir + "CAN";

		 string radar_1_dir =  "./" + radar_1_dest;
		 string radar_2_dir =  "./" + radar_2_dest;
		 string radar_bf_dir =  "./" + radar_bf_dest;
		 string lidar_dir=  "./" + lidar_dest ;
		 string camera_dir_1 = "./" + camera_dest_1;
		string camera_dir_2 = "./" + camera_dest_2;
		string camera_dir_3 = "./" + camera_dest_3;
		string camera_dir = "./" + camera_dest;
		string GPS_dir = "./" + GPS_dest;
		string IMU_dir = "./" +  IMU_dest;
		string CAN_dir = "./" +  CAN_dest;

		union Float xf,yf,zf;
		union Double tD;
		union Uint16 uRing;
		uint8_t intensity;
		int radar1FrameNum = 0;
		int radar2FrameNum = 0;
		int radar_bf_FrameNum = 0;
		int lidarFrameNum = 0;
		int imageFrameNum1 = 0;
		int imageFrameNum2 = 0;
		int imageFrameNum3 = 0;
		int imuFrameNum = 0;
		int gpsFrameNum = 0;
		int canFrameNum = 0;

		   string meta_data_file_name = "./" +  radar_bf_dest  + "_meta_data" + ".txt";

		   string radar_1_timetag_file_name =   "./" +  radar_1_dest  + "_time_tag" + ".txt";
		   string radar_2_timetag_file_name =   "./" +  radar_2_dest  + "_time_tag" + ".txt";
		   string radar_bf_timetag_file_name =  "./" +  radar_bf_dest  + "_time_tag" + ".txt";
		   string lidar_timetag_file_name =  "./" +  lidar_dest  + "_time_tag" + ".txt";
		   string cam1_timetag_file_name =  "./" +  camera_dest_1 + "_time_tag" + ".txt";
		   string cam2_timetag_file_name =  "./" +  camera_dest_2 + "_time_tag" + ".txt";
		   string cam3_timetag_file_name =  "./" +  camera_dest_3 + "_time_tag" + ".txt";
		   string cam_timetag_file_name =  "./" +  camera_dest + "_time_tag" + ".txt";
		   string GPS_timetag_file_name =  "./" +  GPS_dest  + "_time_tag" + ".txt";
		   string  IMU_timetag_file_name =  "./" +  IMU_dest  + "_time_tag" + ".txt";
           string  CAN_timetag_file_name =  "./" +  CAN_dest  + "_time_tag" + ".txt";
		  
		 

		   ofstream radar_1_timetag;
		   ofstream radar_2_timetag;
		   ofstream radar_bf_timetag;
		   ofstream radar_metadata (meta_data_file_name);
		   ofstream lidar_timetag;
		   ofstream cam1_timetag;
		   ofstream cam2_timetag;
		   ofstream cam3_timetag;
		   ofstream IMU_timetag;
		   ofstream GPS_timetag;
           ofstream CAN_timetag;

		   radar_metadata<< "Mode" << " = " <<  detected_mode << endl;

		   radar_metadata.close();

     foreach (rosbag::MessageInstance const m, view)
    {

        if ( std::strcmp( m.getTopic().c_str(), "/radar") == 0)
        {

			int  radar_ch_cntr = 0;
			
			 if (radar_1_exist == false)
			 {
				 radar_1_exist = true; 
				 mkdir( (radar_1_dest).c_str() , 0777);
				 radar_1_timetag.open(radar_1_timetag_file_name);
			 }

           sensor_msgs::PointCloud2::Ptr ptcld = m.instantiate<sensor_msgs::PointCloud2>();

		   string radar_1_file_name =  radar_1_dir + "/" + std::to_string(radar1FrameNum) + "_.txt";

			
		   ofstream radar_1_file (radar_1_file_name);

		  // cout<< "Timestamp: " << ptcld->header.stamp <<endl;
		    radar_1_timetag <<  radar1FrameNum << "," <<ptcld->header.stamp <<endl;

			radar1FrameNum++;
			  

		  for(int i = 0,j=0 ; i < ptcld->width; i++,j=j+50)
			{
				union Float xx , yy , zz , rcs ,velocity ,  temp,  elev_indx , vel_indx ;  
				union  Uint16  rang_indx , thetha_indx, PacketId  ;
				union  Uint32 amp_indx,timestamp ,  FrameId;
				
				  xx.m_bytes[0] = ptcld->data[j] ;
				  xx.m_bytes[1] = ptcld->data[j+1] ;
				  xx.m_bytes[2] = ptcld->data[j+2] ;
				  xx.m_bytes[3] = ptcld->data[j+3] ;
			
				yy.m_bytes[0] = ptcld->data[j+4] ;
				yy.m_bytes[1] = ptcld->data[j+5] ;
				yy.m_bytes[2] = ptcld->data[j+6] ;
				yy.m_bytes[3] = ptcld->data[j+7] ;
			
				  zz.m_bytes[0] = ptcld->data[j+8] ;
				  zz.m_bytes[1] = ptcld->data[j+9] ;
				  zz.m_bytes[2] = ptcld->data[j+10] ;
                  zz.m_bytes[3] = ptcld->data[j+11] ;
				
				  rcs.m_bytes[0]= ptcld->data[j+12] ;
				  rcs.m_bytes[1]= ptcld->data[j+13] ;
				  rcs.m_bytes[2]= ptcld->data[j+14] ;
                  rcs.m_bytes[3]= ptcld->data[j+15] ;
			
				   velocity.m_bytes[0]= ptcld->data[j+16] ;
				   velocity.m_bytes[1]= ptcld->data[j+17] ;
				   velocity.m_bytes[2]= ptcld->data[j+18] ;
                   velocity.m_bytes[3]= ptcld->data[j+19] ;
				
				   rang_indx.m_bytes[0]= ptcld->data[j+20] ;
				   rang_indx.m_bytes[1]= ptcld->data[j+21] ;
				
				
				   thetha_indx.m_bytes[0]= ptcld->data[j+22] ;
				   thetha_indx.m_bytes[1]= ptcld->data[j+23] ;
			
			
				   elev_indx.m_bytes[0]= ptcld->data[j+24] ;
				   elev_indx.m_bytes[1]= ptcld->data[j+25] ;
				   elev_indx.m_bytes[2]= ptcld->data[j+26] ;
				   elev_indx.m_bytes[3]= ptcld->data[j+27] ;
			
			       vel_indx.m_bytes[0]= ptcld->data[j+28] ;
				   vel_indx.m_bytes[1]= ptcld->data[j+29] ;
				   vel_indx.m_bytes[2]= ptcld->data[j+30] ;
				   vel_indx.m_bytes[3]= ptcld->data[j+31] ;
			
				   amp_indx.m_bytes[0]= ptcld->data[j+32] ;
				   amp_indx.m_bytes[1]= ptcld->data[j+33] ;
				  amp_indx.m_bytes[2]= ptcld->data[j+34] ;
                  amp_indx.m_bytes[3]= ptcld->data[j+35] ;
			
				   timestamp.m_bytes[0]= ptcld->data[j+36] ;
				   timestamp.m_bytes[1]= ptcld->data[j+37] ;
				   timestamp.m_bytes[2]= ptcld->data[j+38] ;
                   timestamp.m_bytes[3]= ptcld->data[j+39] ;
				
				   temp.m_bytes[0]= ptcld->data[j+40] ;
				   temp.m_bytes[1]= ptcld->data[j+41] ;
				   temp.m_bytes[2]= ptcld->data[j+42] ;
                   temp.m_bytes[3]= ptcld->data[j+43] ;
			
				  FrameId.m_bytes[0]= ptcld->data[j+44] ;
				  FrameId.m_bytes[1]= ptcld->data[j+45] ;
				  FrameId.m_bytes[2]= ptcld->data[j+46] ;
                  FrameId.m_bytes[3]= ptcld->data[j+47] ;
			
				  PacketId.m_bytes[0]= ptcld->data[j+48] ;
				  PacketId.m_bytes[1]= ptcld->data[j+49] ;		
			 

			   float r = sqrt((xx.m_float * xx.m_float) + (yy.m_float* yy.m_float) + (zz.m_float* zz.m_float));
			   float azimuth = 90 + (atan2(-yy.m_float,xx.m_float)* (180 / PI));
			   float elev = atan2(zz.m_float,r)* (180 / PI);

			   int pos = radar_ch_cntr;
			   radar_ch_cntr += sprintf(radar_1_buffer+pos,"%5u , %5u , %8.4f , %8.4f  , %8.4f , %8.4f , %8.4f  , %8.4f ,%8.4f , %+05.4f  , %5d , %4d , %8.4f  , %8.4f , %8u , %8u  , %8.4f \r\n",FrameId.m_uint32,PacketId.m_uint16,xx.m_float , yy.m_float, zz.m_float, r, azimuth, elev, rcs.m_float, velocity.m_float, rang_indx.m_uint16,thetha_indx.m_uint16, elev_indx.m_float, vel_indx.m_float, amp_indx.m_uint32, timestamp.m_uint32, temp.m_float);

		       //radar_bf_file<< std::setprecision(5)<<FrameId<<","<<PacketId<<","<<xx<<","<<yy<<","<<zz<<"," << r<< "," << azimuth<< "," << elev<< ","<<rcs<<","<<velocity<<","<<rang_indx<<","<<  thetha_indx<<","<<elev_indx<<","<<vel_indx<<","<<amp_indx<<","<<timestamp<<","<<temp<<endl;
			}
			//cout<<"radar_ch_cntr   =  " <<radar_ch_cntr <<endl;
			radar_1_file.write (radar_1_buffer,radar_ch_cntr);
			radar_1_file.close();

        }
		 else if ( std::strcmp( m.getTopic().c_str(), "/radar_bf") == 0)
        {
			  int  radarBF_ch_cntr = 0;
			  
			  if (radar_bf_exist == false)
			 {
				 radar_bf_exist = true; 
				 mkdir( (radar_bf_dest).c_str() , 0777);
				 radar_bf_timetag.open(radar_bf_timetag_file_name);
			 } 

		   sensor_msgs::PointCloud2::Ptr ptc2ld = m.instantiate<sensor_msgs::PointCloud2>();

		   string radar_bf_file_name =  radar_bf_dir + "/" + std::to_string(radar_bf_FrameNum) + "_.txt";

		   ofstream radar_bf_file (radar_bf_file_name);
		   
          cout<< "Radar Frame: " << radar_bf_FrameNum <<endl;
		  
		    radar_bf_timetag <<  radar_bf_FrameNum << "," <<ptc2ld->header.stamp <<endl;

			 radar_bf_FrameNum++;

		   	   for(int i = 0,j=0 ; i < ptc2ld->width; i++,j=j+50)
			{
				union Float xx , yy , zz , rcs ,velocity ,  temp , elev_indx , vel_indx ;  
				union  Uint16  rang_indx , thetha_indx, PacketId  ;
				union  Uint32 amp_indx,timestamp ,  FrameId;
				
				
				  xx.m_bytes[0] = ptc2ld->data[j] ;
				  xx.m_bytes[1] = ptc2ld->data[j+1] ;
				  xx.m_bytes[2] = ptc2ld->data[j+2] ;
				  xx.m_bytes[3] = ptc2ld->data[j+3] ;
			
				yy.m_bytes[0] = ptc2ld->data[j+4] ;
				yy.m_bytes[1] = ptc2ld->data[j+5] ;
				yy.m_bytes[2] = ptc2ld->data[j+6] ;
				yy.m_bytes[3] = ptc2ld->data[j+7] ;
			
				  zz.m_bytes[0] = ptc2ld->data[j+8] ;
				  zz.m_bytes[1] = ptc2ld->data[j+9] ;
				  zz.m_bytes[2] = ptc2ld->data[j+10] ;
                  zz.m_bytes[3] = ptc2ld->data[j+11] ;
				
				  rcs.m_bytes[0]= ptc2ld->data[j+12] ;
				  rcs.m_bytes[1]= ptc2ld->data[j+13] ;
				  rcs.m_bytes[2]= ptc2ld->data[j+14] ;
                  rcs.m_bytes[3]= ptc2ld->data[j+15] ;
			
				   velocity.m_bytes[0]= ptc2ld->data[j+16] ;
				   velocity.m_bytes[1]= ptc2ld->data[j+17] ;
				   velocity.m_bytes[2]= ptc2ld->data[j+18] ;
                   velocity.m_bytes[3]= ptc2ld->data[j+19] ;
				
				   rang_indx.m_bytes[0]= ptc2ld->data[j+20] ;
				   rang_indx.m_bytes[1]= ptc2ld->data[j+21] ;
				
				
				   thetha_indx.m_bytes[0]= ptc2ld->data[j+22] ;
				   thetha_indx.m_bytes[1]= ptc2ld->data[j+23] ;
			
			
				   elev_indx.m_bytes[0]= ptc2ld->data[j+24] ;
				   elev_indx.m_bytes[1]= ptc2ld->data[j+25] ;
				   elev_indx.m_bytes[2]= ptc2ld->data[j+26] ;
				   elev_indx.m_bytes[3]= ptc2ld->data[j+27] ;
			
			       vel_indx.m_bytes[0]= ptc2ld->data[j+28] ;
				   vel_indx.m_bytes[1]= ptc2ld->data[j+29] ;
				   vel_indx.m_bytes[2]= ptc2ld->data[j+30] ;
				   vel_indx.m_bytes[3]= ptc2ld->data[j+31] ;
			
				   amp_indx.m_bytes[0]= ptc2ld->data[j+32] ;
				   amp_indx.m_bytes[1]= ptc2ld->data[j+33] ;
				  amp_indx.m_bytes[2]= ptc2ld->data[j+34] ;
                  amp_indx.m_bytes[3]= ptc2ld->data[j+35] ;
			
				   timestamp.m_bytes[0]= ptc2ld->data[j+36] ;
				   timestamp.m_bytes[1]= ptc2ld->data[j+37] ;
				   timestamp.m_bytes[2]= ptc2ld->data[j+38] ;
                   timestamp.m_bytes[3]= ptc2ld->data[j+39] ;
				
				   temp.m_bytes[0]= ptc2ld->data[j+40] ;
				   temp.m_bytes[1]= ptc2ld->data[j+41] ;
				   temp.m_bytes[2]= ptc2ld->data[j+42] ;
                   temp.m_bytes[3]= ptc2ld->data[j+43] ;
			
				  FrameId.m_bytes[0]= ptc2ld->data[j+44] ;
				  FrameId.m_bytes[1]= ptc2ld->data[j+45] ;
				  FrameId.m_bytes[2]= ptc2ld->data[j+46] ;
                  FrameId.m_bytes[3]= ptc2ld->data[j+47] ;
			
				  PacketId.m_bytes[0]= ptc2ld->data[j+48] ;
				  PacketId.m_bytes[1]= ptc2ld->data[j+49] ;			
			 

			   float r = sqrt((xx.m_float * xx.m_float) + (yy.m_float* yy.m_float)+ (zz.m_float* zz.m_float));
			   float azimuth = 90 + (atan2(-yy.m_float,xx.m_float)* (180 / PI));
			     float elev = atan2(zz.m_float,r)* (180 / PI);

			   int pos = radarBF_ch_cntr;
			   radarBF_ch_cntr += sprintf(radarBF_buffer+pos,"%5u , %5u , %8.4f , %8.4f  , %8.4f , %8.4f , %8.4f  , %8.4f ,%8.4f , %+05.4f  , %5d , %4d , %8.4f  , %8.4f , %8u , %8u  , %8.4f \r\n",FrameId.m_uint32,PacketId.m_uint16,xx.m_float , yy.m_float, zz.m_float, r, azimuth, elev, rcs.m_float, velocity.m_float, rang_indx.m_uint16,thetha_indx.m_uint16, elev_indx.m_float, vel_indx.m_float, amp_indx.m_uint32, timestamp.m_uint32, temp.m_float);

		       //radar_bf_file<< std::setprecision(5)<<FrameId<<","<<PacketId<<","<<xx<<","<<yy<<","<<zz<<"," << r<< "," << azimuth<< "," << elev<< ","<<rcs<<","<<velocity<<","<<rang_indx<<","<<  thetha_indx<<","<<elev_indx<<","<<vel_indx<<","<<amp_indx<<","<<timestamp<<","<<temp<<endl;

			}
			radar_bf_file.write (radarBF_buffer,radarBF_ch_cntr);
			radar_bf_file.close();

        }
        else if ( std::strcmp( m.getTopic().c_str(), "/pandar40p") == 0)
		{

			int  lidar_ch_cntr = 0;
			
			  if (lidar_exist == false)
			 {
				 lidar_exist = true; 
				 mkdir( (lidar_dest).c_str() , 0777);
				 lidar_timetag.open(lidar_timetag_file_name);
			 }

		   sensor_msgs::PointCloud2::Ptr ptc2ld = m.instantiate<sensor_msgs::PointCloud2>();

		   string lidar_file_name = lidar_dir + "/" + std::to_string(lidarFrameNum) + "_.txt";


		   ofstream lidar_file (lidar_file_name);

		   string bk = "./";


		   //cout<< "Timestamp: " << ptc2ld->header.stamp <<endl;
		   
		    lidar_timetag << lidarFrameNum<< ","<<m.getTime()<<endl;
		   
		   /*  for(int i = 0 ; i < 6; ++i)
			{

		   cout<< i  << "," <<ptc2ld->fields[i]<<endl;
			}
			*/
			lidarFrameNum++;

		   for(int i = 0,j=0 ; i < ptc2ld->width; i++,j=j+48)
			{

		    xf.m_bytes[0] = ptc2ld->data[j];
			xf.m_bytes[1] = ptc2ld->data[j+1];
		    xf.m_bytes[2] = ptc2ld->data[j+2];
		    xf.m_bytes[3] = ptc2ld->data[j+3];

			yf.m_bytes[0] = ptc2ld->data[j+4];
			yf.m_bytes[1] = ptc2ld->data[j+5];
		    yf.m_bytes[2] = ptc2ld->data[j+6];
			yf.m_bytes[3] = ptc2ld->data[j+7];


			zf.m_bytes[0] = ptc2ld->data[j+8];
			zf.m_bytes[1] = ptc2ld->data[j+9];
		    zf.m_bytes[2] = ptc2ld->data[j+10];
			zf.m_bytes[3] = ptc2ld->data[j+11];

			intensity =  ptc2ld->data[j+16];


			tD.m_bytes[0] = ptc2ld->data[j+24];
		    tD.m_bytes[1] = ptc2ld->data[j+25];
			tD.m_bytes[2] = ptc2ld->data[j+26];
		    tD.m_bytes[3] = ptc2ld->data[j+27];
			tD.m_bytes[4] = ptc2ld->data[j+28];
			tD.m_bytes[5] = ptc2ld->data[j+29];
			tD.m_bytes[6] = ptc2ld->data[j+30];
			tD.m_bytes[7] = ptc2ld->data[j+31];

			uRing.m_bytes[0] = ptc2ld->data[j+32];
			uRing.m_bytes[1] = ptc2ld->data[j+33];


			int pos = lidar_ch_cntr;
			   lidar_ch_cntr += sprintf(lidar_buffer+pos,"%8.4f , %8.4f  , %8.4f , %4u , %8.4f  , %5d \r\n",xf.m_float, yf.m_float, zf.m_float, intensity, tD.m_double, uRing.m_uint16);
				//lidar_file<<std::setprecision(5)<< xf.m_float<<","<<yf.m_float<<","<<zf.m_float<<","<<std::to_string(intensity)<<"," << tD.m_double<< ","<< uRing.m_uint16<< endl;
			    //cout<< i  << "," <<std::setprecision(5)<< xf.m_float<<","<<yf.m_float<<","<<zf.m_float<<","<<std::to_string(intensity)<<"," << tD.m_double<<","<< uRing.m_uint16<<endl;
			}
			lidar_file.write (lidar_buffer, lidar_ch_cntr);
			lidar_file.close();

		}
        else if ( std::strcmp( m.getTopic().c_str(), "/camera/cam_1") == 0)
		{
				sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
				
		     if (camera_1_exist == false)
			 {
				 camera_1_exist = true; 
				 mkdir( (camera_dest_1).c_str() , 0777);
				 cam1_timetag.open(cam1_timetag_file_name);
			 }

				Mat mat (l_img->height , l_img->width, CV_8UC3);
				
				int xx = ((l_img->width)*3);

				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * xx) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * xx) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * xx) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}

					std::vector<int> compression_params;

					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);

					string camera_file_name =   camera_dir_1 + "/" + std::to_string(imageFrameNum1) + "_.jpg";

					 cam1_timetag <<  imageFrameNum1<< "," << m.getTime() <<endl;

					bool result = false;

					try
					{
				       result  = imwrite(camera_file_name,mat,compression_params);
					}

					catch(const cv::Exception& ex)
					{
						cout<< "Exception converting image to PNG   " << ex.what()<<endl;
					}

					if (result)
					{
						//printf("Saved PNG file with alpha data.\n");
					}
					else
					{
						printf("ERROR: Can't save PNG file.\n");
					}

					 imageFrameNum1++;

		}
		else if ( std::strcmp( m.getTopic().c_str(), "/camera/cam_2") == 0)
		{
					sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
					
		     if (camera_2_exist == false)
			 {
				 camera_2_exist = true; 
				 mkdir( (camera_dest_2).c_str() , 0777);
				 cam2_timetag.open(cam2_timetag_file_name);
			 }	

				Mat mat (l_img->height , l_img->width, CV_8UC3);
				
				int xx = ((l_img->width)*3);

				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * xx) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * xx) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * xx) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}

					std::vector<int> compression_params;

					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);

					string camera_file_name =   camera_dir_2 + "/" + std::to_string(imageFrameNum2) + "_.jpg";

					cam2_timetag <<  imageFrameNum2<< "," << m.getTime() <<endl;

					bool result = false;

					try
					{
				       result  = imwrite(camera_file_name,mat,compression_params);
					}

					catch(const cv::Exception& ex)
					{
						cout<< "Exception converting image to PNG   " << ex.what()<<endl;
					}

					if (result)
					{
						//printf("Saved PNG file with alpha data.\n");
					}
					else
					{
						printf("ERROR: Can't save PNG file.\n");
					}

					 imageFrameNum2++;

		}
		else if ( std::strcmp( m.getTopic().c_str(), "/camera/cam_3") == 0)
		{
				sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
				
			if (camera_3_exist == false)
			 {
				 camera_3_exist = true; 
				 mkdir( (camera_dest_3).c_str() , 0777);
				 cam3_timetag.open(cam3_timetag_file_name);
			 }	

				Mat mat (l_img->height , l_img->width, CV_8UC3);
				
			     int xx = ((l_img->width)*3);

				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * xx ) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * xx) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * xx) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}

					std::vector<int> compression_params;

					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);

					string camera_file_name =   camera_dir_3 + "/" + std::to_string(imageFrameNum3) + "_.jpg";

					cam3_timetag <<  imageFrameNum3<< "," << m.getTime() <<endl;

					bool result = false;

					try
					{
				       result  = imwrite(camera_file_name,mat,compression_params);
					}

					catch(const cv::Exception& ex)
					{
						cout<< "Exception converting image to PNG   " << ex.what()<<endl;
					}

					if (result)
					{
						//printf("Saved PNG file with alpha data.\n");
					}
					else
					{
						printf("ERROR: Can't save PNG file.\n");
					}

					 imageFrameNum3++;


		}
		else if ( std::strcmp( m.getTopic().c_str(), "/IMU") == 0)
		{
					sensor_msgs::Imu::ConstPtr   imu = m.instantiate<sensor_msgs::Imu>();
					
			  if (IMU_exist == false)
			 {
				 IMU_exist = true; 
				 mkdir( (IMU_dest).c_str() , 0777);
				 IMU_timetag.open(IMU_timetag_file_name);
			 }	

					string IMU_file_name =   IMU_dir + "/" + std::to_string(imuFrameNum) + "_.txt";

					IMU_timetag <<  imuFrameNum<< "," << imu->header.stamp <<endl;

					ofstream IMU_file (IMU_file_name);

					auto tt =  imu->header.stamp;
					float aa= imu->orientation.w;
					float bb= imu->orientation.x;
					float cc= imu->orientation.y;
					float dd= imu->orientation.z;

					float ee= imu->angular_velocity.x ;
					float ff= imu->angular_velocity.y ;
					float gg= imu->angular_velocity.z ;

					float hh= imu->linear_acceleration.x ;
					float ii = imu->linear_acceleration.y ;
					float jj = imu->linear_acceleration.z ;

					float kk,ll,mm,nn,oo,pp;

					float East_Speed = kk = imu->angular_velocity_covariance[0]; 
					float North_Speed = ll = imu->angular_velocity_covariance[1];
					float Vertical_Speed = mm = imu->angular_velocity_covariance[2];
					float GNSS_Horizontal_Speed = nn = imu->angular_velocity_covariance[3];
					float GNSS_Trackover_Ground = oo = imu->angular_velocity_covariance[4];
					float GNSS_Vertical_Speed = pp = imu->angular_velocity_covariance[5];
					IMU_file<<std::setprecision(7)<<tt<<","<<aa<<","<<bb<<","<<cc<<","<<dd<<","<<ee<<","<<ff<<","<<gg<<","<<hh<<","<<ii<<","<<jj<<","<<kk<<","<<ll<<","<<mm<<","<<nn<<","<<oo<<","<<pp<<endl;

					IMU_file.close();

					 imuFrameNum++;

		}
		else if ( std::strcmp( m.getTopic().c_str(), "/GPS") == 0)
		{

					sensor_msgs::NavSatFix::ConstPtr   gps = m.instantiate<sensor_msgs::NavSatFix>();
					
			  if (GPS_exist == false)
			 {
				 GPS_exist = true; 
				 mkdir( (GPS_dest).c_str() , 0777);
				 GPS_timetag.open(GPS_timetag_file_name);
			 }	

					string GPS_file_name =   GPS_dir + "/" + std::to_string(gpsFrameNum) + "_.txt";

					GPS_timetag <<  gpsFrameNum<< "," << gps->header.stamp <<endl;

					ofstream GPS_file (GPS_file_name);

					auto tt =  gps->header.stamp;
				    float  lat = gps->latitude ;
                    float  lon = gps->longitude;
                    float alt = gps->altitude;


					GPS_file<<std::setprecision(12)<< tt <<","<< lat<<","<< lon <<","<< alt<<endl;

					GPS_file.close();

					 gpsFrameNum++;

		}
#ifdef CAN
	        else if ( std::strcmp( m.getTopic().c_str(), "/CAN_msg") == 0)
		{

					can_msgs::Frame::ConstPtr   can = m.instantiate<can_msgs::Frame>();
					
			if (CAN_exist == false)
			 {
				 CAN_exist = true; 
				 mkdir( (CAN_dest).c_str() , 0777);
				 CAN_timetag.open(CAN_timetag_file_name);
			 }	

					string CAN_file_name =   CAN_dir + "/" + std::to_string(canFrameNum) + "_.txt";

					CAN_timetag <<  canFrameNum<< "," << can->header.stamp <<endl;

					ofstream CAN_file(CAN_file_name);

					auto tt =  can->header.stamp;

			                string speedKmHr = std::to_string( can->data[3]);
 					string speedMiHr  = std::to_string((can->data[3])/1.609);

					CAN_file <<std::setprecision(12)<< tt <<","<<speedKmHr<<","<< speedMiHr <<endl;
                                        //cout <<std::setprecision(12)<< tt <<","<<speedKmHr<<","<< speedMiHr <<endl;
  					//printf("\r\n %f   %X    %X    %X    %X ", tt , can->data[0] , can->data[1] , can->data[2] , can->data[3] );
					CAN_file.close();

					canFrameNum++;

		}
#endif	

        if( (imageFrameNum1 > k )&& (imageFrameNum2 > k )&& (imageFrameNum3 > k))
		{
			
			 if (camera_exist == false)
			 {
				 camera_exist = true; 
				 mkdir( (camera_dest).c_str() , 0777);
			 }	
			 
        int ret = Concat( bag_wthExtDir , k)	;

		if(ret == 0)
		{
			k++;
		}

		}


    }// foreach

	lidar_timetag.close();
	radar_1_timetag.close();
	radar_2_timetag.close();
	radar_bf_timetag.close();
	cam1_timetag.close();
	cam2_timetag.close();
	cam3_timetag.close();
	IMU_timetag.close();
	GPS_timetag.close();
	
#ifdef CAN	
    CAN_timetag.close();
#endif	
	

	char buffer2[80];
	time_t now2 = time(0);
	tm *ltm2 = localtime(&now2);

    strftime(buffer2, 80, "%Y/%m/%d_%H:%M:%S", ltm2);
    cout<< buffer2<<endl;

	bag.close();


	 if (camera_exist == true)
	 {
	 string cmd =  " rm -rf " + camera_dest_1;
	 cout<<cmd<<endl;
     system(cmd.c_str()); 

	 cmd =  " rm -rf " + camera_dest_2;
	 cout<<cmd<<endl;
     system(cmd.c_str()); 


	 cmd =  " rm -rf " + camera_dest_3;
	 cout<<cmd<<endl;
     system(cmd.c_str());
	 }
	 else
	 {
		string cmd =  "mv " + camera_dest_2 + " " +  camera_dest;
	    cout<<cmd<<endl;
        system(cmd.c_str()); 
	 //   cmd =  "mv  " + cam2_timetag_file_name  + " " + cam_timetag_file_name;
	//    cout<<cmd<<endl;
     //   system(cmd.c_str()); 
	 }



     int result = 0;
     string cmd =  " tar -cvf " +  bag_wthExt + ".tar  " +  bag_wthExt;
	 cout<<cmd<<endl;
    result = system(cmd.c_str());
	
	cout << endl<<" taring result : " << result<<endl;
	 
	 cmd = " rm -rf " +  bag_wthExt;
	 cout<<cmd<<endl;
     system(cmd.c_str());
	 
	 
	}

	// string dest_dir = "/run/user/1000/gvfs/afp-volume:host=MassStorage.local,user=Ziran.Wu,volume=MassStorage/ROS/DataCollection/";
	string dest_dir = "/mnt/y/ROS/DataCollection/";
	 string src_dir = current_path();
	 
	 
     int result = 0;
	string cmd = " cp -r " + src_dir + " " + dest_dir;
	cout<<cmd<<endl;
	result = system(cmd.c_str());
	cout << endl<<" copying result : " << result<<endl;
	
	if(result != 0)
	{
		return 0;
	}
	
	
	system("rm *.tar");

	 std::size_t found = src_dir.find_last_of("/\\");
	 cout<<src_dir.substr(found+1)<<endl;

	 dest_dir = dest_dir + src_dir.substr(found+1);
	cout<<dest_dir<<endl;

	sleep(3);

	 cmd = "cd " + dest_dir;
	 cout<<cmd<<endl;
	 system(cmd.c_str());

   string dest_dir_tar = dest_dir + "/" + "*.tar";
	cout<<dest_dir_tar<<endl;

	 int err2 = glob((dest_dir_tar).c_str(), 0, NULL,  &globbuf);
   	if(err2 == 0)
     {
	 	for (size_t i = 0; i < globbuf.gl_pathc; i++)
	 	{
	 	  dest_fileList.push_back(globbuf.gl_pathv[i]);
	 	}

      globfree(&globbuf);
     }

	 if(dest_fileList.size() == 0)
	 {
	 	std::cout<< " No tar file is found. exit the program."<<std::endl;
	 	return 0;
	 }


	 for(auto it = dest_fileList.begin() ; it != dest_fileList.end(); it++)
	 {
	    string tar_file = (*it);
	    cout<<tar_file<<endl;
	    cmd = "tar -xvf " + tar_file + " -C " + dest_dir;
	    cout<<cmd<<endl;
	    system(cmd.c_str());
	 }

	 cmd = "rm " + dest_dir_tar;
	 cout<<cmd<<endl;
	 system(cmd.c_str());
	exit(1);

    return 0;
}
