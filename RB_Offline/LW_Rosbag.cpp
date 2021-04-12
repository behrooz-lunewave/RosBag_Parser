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
#include <boost/foreach.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/imgcodecs.hpp>

#define foreach BOOST_FOREACH
#define PI 3.14159265
#define BUILD_ID  1004.5

using namespace std;
using namespace cv;


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
	
	string camera_dest_1= bag_file + "_Camera_1";
	string camera_dest_2 = bag_file + "_Camera_2";
	string camera_dest_3 = bag_file + "_Camera_3";


        cv::Mat cam_1 = cv::imread( camera_dest_1 +"/" + std::to_string(i) + "_.png", cv::IMREAD_COLOR);
        cv::Mat cam_2 = cv::imread( camera_dest_2 +"/"+ std::to_string(i) + "_.png", cv::IMREAD_COLOR);
        cv::Mat cam_3 = cv::imread( camera_dest_3 +"/"+ std::to_string(i) + "_.png", cv::IMREAD_COLOR);
        cv::Mat cam_12, cam;

        int image_width = 648;
        int image_height = 496;

        int offset_left_x = 260;
        int offset_left_y = 0;
        int offset_right_x = 235;
        int offset_right_y = 0;

        cv::Mat cam_1_crop;
        cv::Mat cam_3_crop;

        imageCrop(cam_1, cam_1_crop, offset_left_x, offset_left_y, 0);    // crop right side image
        imageCrop(cam_3, cam_3_crop, offset_right_x, offset_right_y, 1);  // crop left side image

        cv::hconcat(cam_1_crop, cam_2, cam_12);
		// cv::hconcat(cam_1, cam_2, cam_12);
		  cv::hconcat(cam_12, cam_3_crop, cam);
        //cv::hconcat(cam_12, cam_3, cam);

       // cv::imshow("All cameras", cam);
	   
	   	try
		{		
				cv::imwrite(  bag_file + "_Camera/concat _" + std::to_string(i) + ".png",  cam);
		}			
		catch(const cv::Exception& ex)
		{
				cout<< "Exception converting image to PNG  after concatanation  for image  " <<i<<"   "<<endl<< ex.what()<<endl;
		}
		

    return 0;

}


char radar_buffer[1024*1024];
char radarBF_buffer[1024*1024];
char lidar_buffer[1024*1024*64];

int main (int argc, char **argv) 
{ 
    glob_t globbuf;
	ros::Time::init();
	vector<string> fileList;
	
	rosbag::Bag bag;
	
	vector<string> ModeSreach = { "64N","64F","128N","128F","256N","256F"};
		  
	//string bag_file 	 = std::to_string(argv[1] );
	
/*	if(argc < 2)
	{
		std::cout<< " Error. Please Enter the name the rosbag file. For instance:  $ ./radar_txt  test.bag"<<std::endl;	
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


	   bag.open(bag_file, rosbag::bagmode::Read);
      std::vector<std::string> bag_topics;

       bag_topics.push_back(std::string("/radar"));
	   bag_topics.push_back(std::string("/radar_bf"));
       bag_topics.push_back(std::string("/pandar40p"));
       bag_topics.push_back(std::string("/camera/cam_1"));
	   bag_topics.push_back(std::string("/camera/cam_2"));
	   bag_topics.push_back(std::string("/camera/cam_3"));
	   bag_topics.push_back(std::string("/IMU"));
	   //bag_topics.push_back(std::string("/GPS"));
	  
	   
	rosbag::View view(bag, rosbag::TopicQuery(bag_topics));
   
  
	
		//ros::Rate loop_rate(20);
		
		//int i =0;
		
		string bag_wthExt = bag_file.substr(0, bag_file.size() - 4);
		
		mkdir(bag_wthExt.c_str(), 0777);
		
		string bag_wthExtDir = bag_wthExt + "/" + bag_wthExt ;
	
		string radar_dest = bag_wthExtDir+ "_Radar";
		string radar_bf_dest = bag_wthExtDir+ "_Radar_bf";
		string lidar_dest = bag_wthExtDir + "_Lidar";
		string camera_dest_1= bag_wthExtDir+ "_Camera_1";
		string camera_dest_2 = bag_wthExtDir+ "_Camera_2";
		string camera_dest_3 = bag_wthExtDir + "_Camera_3";
		string camera_dest = bag_wthExtDir + "_Camera";
		//string GPS_dest = bag_wthExtDir + "_GPS";
		string IMU_dest = bag_wthExtDir + "_IMU";
		 
		 mkdir( (radar_dest).c_str() , 0777);
		 mkdir( (radar_bf_dest ).c_str() , 0777);
		 mkdir( (lidar_dest ).c_str() , 0777);
		 mkdir( (camera_dest_1).c_str() , 0777);
		 mkdir( (camera_dest_2).c_str() , 0777);
		 mkdir( (camera_dest_3 ).c_str() , 0777);
		 mkdir( (camera_dest ).c_str() , 0777);
		 //mkdir( (GPS_dest ).c_str() , 0777);
		 mkdir( (IMU_dest ).c_str() , 0777);
		   
		 string radar_dir =  "./" + radar_dest;
		 string radar_bf_dir =  "./" + radar_bf_dest;
		 string lidar_dir=  "./" + lidar_dest ;
		 string camera_dir_1 = "./" + camera_dest_1;
		string camera_dir_2 = "./" + camera_dest_2;
		string camera_dir_3 = "./" + camera_dest_3;
		string camera_dir = "./" + camera_dest;
		//string GPS_dir = "./" + GPS_dest;
		string IMU_dir = "./" +  IMU_dest;
		
		union Float xf,yf,zf;
		union Double tD;
		union Uint16 uRing;
		uint8_t intensity;
		int radarFrameNum = 0;
		int radar_bf_FrameNum = 0;
		int lidarFrameNum = 0;
		int imageFrameNum1 = 0; 
		int imageFrameNum2 = 0; 
		int imageFrameNum3 = 0; 
		int imuFrameNum = 0; 
		
		
		   string meta_data_file_name = "./" +  radar_dest  + "_meta_data" + ".txt";
		   
		   string radar_timetag_file_name =   "./" +  radar_dest  + "_time_tag" + ".txt";
		   string radar_bf_timetag_file_name =  "./" +  radar_bf_dest  + "_time_tag" + ".txt";
		   string lidar_timetag_file_name =  "./" +  lidar_dest  + "_time_tag" + ".txt";
		   string cam1_timetag_file_name =  "./" +  camera_dest_1 + "_time_tag" + ".txt";
		   string cam2_timetag_file_name =  "./" +  camera_dest_2 + "_time_tag" + ".txt";
		   string cam3_timetag_file_name =  "./" +  camera_dest_3 + "_time_tag" + ".txt";
		   //string GPS_timetag_file_name =  "/" +  GPS_dest  + "_time_tag" + ".txt";
		   string  IMU_timetag_file_name =  "./" +  IMU_dest  + "_time_tag" + ".txt";
		  
		   cout<<radar_timetag_file_name<<endl;
		   
		   ofstream radar_timetag (radar_timetag_file_name); 
		   ofstream radar_bf_timetag (radar_bf_timetag_file_name); 
		   
		   ofstream radar_metadata (meta_data_file_name); 
		   
		   ofstream lidar_timetag (lidar_timetag_file_name); 
		   ofstream cam1_timetag (cam1_timetag_file_name); 
		   ofstream cam2_timetag (cam2_timetag_file_name); 
		   ofstream cam3_timetag (cam3_timetag_file_name); 
		   //ofstream GPS_timetag (GPS_timetag_file_name); 
		   ofstream IMU_timetag (IMU_timetag_file_name); 
		   
		   //cout<<"here"<<endl;
		   
		   radar_metadata<< "Mode" << " = " <<  detected_mode << endl;
		   
		   radar_metadata.close();
		
     foreach (rosbag::MessageInstance const m, view)
    {
                      //std::cout<< "Topic:  = "  << m.getTopic().c_str() << std::endl;
					  
        if ( std::strcmp( m.getTopic().c_str(), "/radar") == 0)
        {
			
			  int  radar_ch_cntr = 0;
			
           sensor_msgs::PointCloud::Ptr ptcld = m.instantiate<sensor_msgs::PointCloud>();
		   
		   string radar_file_name =  radar_dir + "/" + std::to_string(radarFrameNum) + "_.txt";
		   	  
			     cout<< "radar_file_name: " << radar_file_name <<endl;
		   ofstream radar_file (radar_file_name); 
		   
		  // cout<< "Timestamp: " << ptcld->header.stamp <<endl;
		    radar_timetag <<  radarFrameNum << "," <<ptcld->header.stamp <<endl;
			
			  radarFrameNum++;
		   
		   for(int i = 0 ; i < ptcld->points.size(); ++i)
			{
			  
			   float xx = ptcld->points[i].x;
			   float yy = ptcld->points[i].y;
			   float zz = ptcld->points[i].z;
			   float rcs = ptcld->channels[0].values[i];
			   float velocity = ptcld->channels[1].values[i];
			   int rang_indx = ptcld->channels[2].values[i];
			   int thetha_indx = ptcld->channels[3].values[i];
			   int elev_indx = ptcld->channels[4].values[i];
			   int vel_indx = ptcld->channels[5].values[i];
			   int amp_indx = ptcld->channels[6].values[i];
			   int timestamp = ptcld->channels[7].values[i];
			   float temp = ptcld->channels[8].values[i];
			   int FrameId =  ptcld->channels[9].values[i];
			   int PacketId =  ptcld->channels[10].values[i];
			   
			   float r = sqrt((xx * xx) + (yy * yy));
			   float azimuth = 90 + (atan2(yy,xx)* (180 / PI));
			   float elev = atan2(zz,r);
			  
		       //radar_file<< std::setprecision(5)<<FrameId<<","<<PacketId<<","<<xx<<","<<yy<<","<<zz<<"," << r<< "," << azimuth<< "," << elev<< ","<<rcs<<","<<velocity<<","<<rang_indx<<","<<  thetha_indx<<","<<elev_indx<<","<<vel_indx<<","<<amp_indx<<","<<timestamp<<","<<temp<<endl;
			   int pos = radar_ch_cntr;
			   radar_ch_cntr += sprintf(radar_buffer+pos,"%5u , %5u , %8.4f , %8.4f  , %8.4f , %8.4f , %8.4f  , %8.4f ,%8.4f , %+05.4f  , %5d , %4d , %4d  , %4d , %8d , %8d  , %8.4f \r\n",FrameId,PacketId,xx,yy,zz, r, azimuth, elev,rcs,velocity,rang_indx, thetha_indx,elev_indx,vel_indx,amp_indx,timestamp,temp);
			   //cout<< "radar_ch_cntr   =  " <<radar_ch_cntr <<endl;
			}
			//cout<< "radar_ch_cntr   =  " <<radar_ch_cntr <<endl;
			radar_file.write (radar_buffer,radar_ch_cntr);
			radar_file.close();

        }  
		 else if ( std::strcmp( m.getTopic().c_str(), "/radar_bf") == 0)
        {
			  int  radarBF_ch_cntr = 0;
           
		   sensor_msgs::PointCloud::Ptr ptcld = m.instantiate<sensor_msgs::PointCloud>();
		   
		   string radar_bf_file_name =  radar_bf_dir + "/" + std::to_string(radar_bf_FrameNum) + "_.txt";
		   	   
		   ofstream radar_bf_file (radar_bf_file_name); 
		   
		  // cout<< "Timestamp: " << ptcld->header.stamp <<endl;
		    radar_bf_timetag <<  radar_bf_FrameNum << "," <<ptcld->header.stamp <<endl;
			
			 radar_bf_FrameNum++;
		   
		   for(int i = 0 ; i < ptcld->points.size(); ++i)
			{
			   float xx = ptcld->points[i].x;
			   float yy = ptcld->points[i].y;
			   float zz = ptcld->points[i].z;
			   float rcs = ptcld->channels[0].values[i];
			   float velocity = ptcld->channels[1].values[i];
			   int rang_indx = ptcld->channels[2].values[i];
			   int thetha_indx = ptcld->channels[3].values[i];
			   int elev_indx = ptcld->channels[4].values[i];
			   int vel_indx = ptcld->channels[5].values[i];
			   int amp_indx = ptcld->channels[6].values[i];
			   int timestamp = ptcld->channels[7].values[i];
			   float temp = ptcld->channels[8].values[i];
			    int FrameId =  ptcld->channels[9].values[i];
			   int PacketId =  ptcld->channels[10].values[i];
			   
			   float r = sqrt((xx * xx) + (yy * yy));
			   float azimuth = 90 + (atan2(yy,xx)* (180 / PI));
			   float elev = atan2(zz,r);
			   
			   int pos = radarBF_ch_cntr;
			   radarBF_ch_cntr += sprintf(radarBF_buffer+pos,"%5u , %5u , %8.4f , %8.4f  , %8.4f , %8.4f , %8.4f  , %8.4f ,%8.4f , %+05.4f  , %5d , %4d , %4d  , %4d , %8d , %8d  , %8.4f \r\n",FrameId,PacketId,xx,yy,zz, r, azimuth, elev,rcs,velocity,rang_indx, thetha_indx,elev_indx,vel_indx,amp_indx,timestamp,temp);
			   
		       //radar_bf_file<< std::setprecision(5)<<FrameId<<","<<PacketId<<","<<xx<<","<<yy<<","<<zz<<"," << r<< "," << azimuth<< "," << elev<< ","<<rcs<<","<<velocity<<","<<rang_indx<<","<<  thetha_indx<<","<<elev_indx<<","<<vel_indx<<","<<amp_indx<<","<<timestamp<<","<<temp<<endl;
			 
			}
			radar_bf_file.write (radarBF_buffer,radarBF_ch_cntr);
			radar_bf_file.close();

        }  
        else if ( std::strcmp( m.getTopic().c_str(), "/pandar40p") == 0)
		{
			
			int  lidar_ch_cntr = 0;
			
		   sensor_msgs::PointCloud2::Ptr ptc2ld = m.instantiate<sensor_msgs::PointCloud2>();
		   
		   string lidar_file_name = lidar_dir + "/" + std::to_string(lidarFrameNum) + "_.txt";
		   
		   
		   ofstream lidar_file (lidar_file_name);
		   
		   string bk = "./";
		     
		   
		   //cout<< "Timestamp: " << ptc2ld->header.stamp <<endl;
		   
		    lidar_timetag << lidarFrameNum<< ","<<ptc2ld->header.stamp <<endl;
		   
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
				
				Mat mat (l_img->height , l_img->width, CV_8UC3);
								
				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * 1944) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * 1944) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * 1944) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}
					
					std::vector<int> compression_params;
					
					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);
					
					string camera_file_name =   camera_dir_1 + "/" + std::to_string(imageFrameNum1) + "_.png";
					
					 cam1_timetag <<  imageFrameNum1<< "," << l_img->header.stamp <<endl;
					
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
					 
					 if( imageFrameNum1%100 == 0)
					 {
						 cout<<endl;
						 cout<< imageFrameNum1<< "  Frames of camera 1 is processed."<<endl;
					 }
		}	
		else if ( std::strcmp( m.getTopic().c_str(), "/camera/cam_2") == 0)	
		{
					sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
				
				Mat mat (l_img->height , l_img->width, CV_8UC3);
								
				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * 1944) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * 1944) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * 1944) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}
					
					std::vector<int> compression_params;
					
					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);
					
					string camera_file_name =   camera_dir_2 + "/" + std::to_string(imageFrameNum2) + "_.png";
					
					cam2_timetag <<  imageFrameNum2<< "," << l_img->header.stamp <<endl;
					
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
					 
					 if( imageFrameNum2%100 == 0)
					 {
						 cout<<endl;
						 cout<< imageFrameNum2<< "  Frames of camera 2 is processed."<<endl;
					 }
				
		}
		else if ( std::strcmp( m.getTopic().c_str(), "/camera/cam_3") == 0)	
		{
									sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
				
				Mat mat (l_img->height , l_img->width, CV_8UC3);
								
				for (int i = 0; i< mat.rows; ++i)
				{
					for (int j = 0; j< mat.cols; ++j)
						{
							Vec3b& bgra = mat.at<Vec3b>(i,j);
							bgra[0] = l_img->data[ (i * 1944) + (j*3) + 0];
							bgra[1] = l_img->data[ (i * 1944) + (j*3) + 1];
							bgra[2] = l_img->data[ (i * 1944) + (j*3) + 2];
							//bgra[3] =  (((bgra[1]+bgra[2] + bgra[3]))/3);
						}
					}
					
					std::vector<int> compression_params;
					
					compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
					compression_params.push_back(9);
					
					string camera_file_name =   camera_dir_3 + "/" + std::to_string(imageFrameNum3) + "_.png";
					
					cam3_timetag <<  imageFrameNum3<< "," << l_img->header.stamp <<endl;
					
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
					 
					 if( imageFrameNum3%100 == 0)
					 {
						 cout<<endl;
						 cout<< imageFrameNum3<< "  Frames of camera 3 is processed."<<endl;
					 }
				
		}
				else if ( std::strcmp( m.getTopic().c_str(), "/IMU") == 0)	
		{
					sensor_msgs::Imu::ConstPtr   imu = m.instantiate<sensor_msgs::Imu>();
						
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
					
					
					IMU_file<<std::setprecision(5)<< tt <<","<< aa <<","<< bb <<","<< cc <<","<< dd <<","<< ee << "," << ff<< "," << gg <<"," << hh <<"," << ii << "," << jj <<endl;
					
					IMU_file.close();
						
					 imuFrameNum++;		
				
		}

        if( (imageFrameNum1 > k )&& (imageFrameNum2 > k )&& (imageFrameNum3 > k))
		{
        Concat( 	bag_wthExtDir , k)	;
		k++;
		}
		
    }// foreach
	
	lidar_timetag.close();
	radar_timetag.close();
	radar_bf_timetag.close();
	cam1_timetag.close();
	cam2_timetag.close();
	cam3_timetag.close();
	IMU_timetag.close();
	
	cout<< imageFrameNum1<< "  Frames of camera 1 is processed."<<endl;
	cout<< imageFrameNum2<< "  Frames of camera 2 is processed."<<endl;
	cout<< imageFrameNum3<< "  Frames of camera 3 is processed."<<endl;
	
	char buffer2[80];
	time_t now2 = time(0);
	tm *ltm2 = localtime(&now2);
	
    strftime(buffer2, 80, "%Y/%m/%d_%H:%M:%S", ltm2);
    cout<< buffer2<<endl;

bag.close();
	}
	 
	 
	 exit(1);
	
    return 0;
}
