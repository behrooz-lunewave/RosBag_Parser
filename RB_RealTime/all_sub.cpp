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


using namespace std;
using namespace cv;

int imageFrameNum1 = 0;
string cam_dir_1 = "Camera_1";
int imageFrameNum2 = 0;
string cam_dir_2 = "Camera_2";
int imageFrameNum3 = 0;
string cam_dir_3 = "Camera_3";

string cam_dir = "Camera";

bool cam1_exist = false;
bool cam2_exist = false;
bool cam3_exist = false;
bool radar_exist = false;
bool radarBF_exist = false;
bool lidar_exist = false;

 ofstream radar_timetag; 
 ofstream radar_bf_timetag; 
 ofstream radar_metadata; 
 ofstream lidar_timetag; 
 ofstream cam1_timetag; 
 ofstream cam2_timetag; 
 ofstream cam3_timetag; 


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

int  Concat(int i) 
{
                                                           //cam_dir_1 + "/" + std::to_string(imageFrameNum1) + "_.jpg";
	
		  
        cv::Mat cam_1 = cv::imread( cam_dir_1 +"/" + std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
        cv::Mat cam_2 = cv::imread( cam_dir_2 +"/"+ std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
        cv::Mat cam_3 = cv::imread( cam_dir_3 +"/"+ std::to_string(i) + "_.jpg", cv::IMREAD_COLOR);
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
			cout<< " Image "<< i << " failed to get concatenated."<< endl;
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
				cv::imwrite(  cam_dir + "/"+ std::to_string(i) + ".jpg",  cam);
		}			
		catch(const cv::Exception& ex)
		{
				cout<< "Exception converting image to PNG  after concatanation  for image  " <<i<<"   "<<endl<< ex.what()<<endl;
		}
		

    return 0;

}


void Cam1_Callback(const sensor_msgs::Image::ConstPtr & l_img)
{
	
	             if(cam1_exist == false)
				 {
					mkdir( (cam_dir_1).c_str() , 0777);
					cam1_exist = true;			
					cam1_timetag.open( "./" +  cam_dir_1 + "_time_tag.txt"); 
				 }
				
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
					compression_params.push_back(0);
					
					string camera_file_name =   cam_dir_1 + "/" + std::to_string(imageFrameNum1) + "_.jpg";
					
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
									
					 imageFrameNum1++;
}


void Cam2_Callback(const sensor_msgs::Image::ConstPtr & l_img)
{
		         if(cam2_exist == false)
				 {
					mkdir( (cam_dir_2).c_str() , 0777);
					cam2_exist = true;		
				    cam2_timetag.open( "./" +  cam_dir_2 + "_time_tag.txt"); 
				 }
				
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
					compression_params.push_back(0);
					
					string camera_file_name =   cam_dir_2 + "/" + std::to_string(imageFrameNum2) + "_.jpg";
					
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
					
					
					 imageFrameNum2++;
  
}

static int k =0;

void Cam3_Callback(const sensor_msgs::Image::ConstPtr & l_img)
{
	
		         if(cam3_exist == false)
				 {
					mkdir( (cam_dir_3).c_str() , 0777);
					cam3_exist = true;		
				    cam3_timetag.open( "./" +  cam_dir_3 + "_time_tag.txt"); 
				 }
				
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
					compression_params.push_back(0);
					
					string camera_file_name =   cam_dir_3 + "/" + std::to_string(imageFrameNum3) + "_.jpg";
					
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
					
					
					 imageFrameNum3++;

}


int lidarFrameNum = 0;
string lidar_dir = "Lidar";
char lidar_buffer[1024*1024*64];

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

union Float xf,yf,zf;
union Double tD;
union Uint16 uRing;
uint8_t intensity;

void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& ptc2ld)
{
 			
				 if(lidar_exist == false)
				 {
					mkdir( (lidar_dir).c_str() , 0777);
					lidar_exist = true;		
				    lidar_timetag.open ( "./" +   lidar_dir + "_time_tag.txt"); 
				 }
			
			int  lidar_ch_cntr = 0;
		   
		   string lidar_file_name = lidar_dir + "/" + std::to_string(lidarFrameNum) + "_.txt";
		   
		   
		   ofstream lidar_file (lidar_file_name);
		   
		      lidar_timetag << lidarFrameNum<< ","<<ptc2ld->header.stamp <<endl;
		   
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

int radarFrameNum = 0;
string radar_dir = "Radar";
char radar_buffer[1024*1024*8];
char radarBF_buffer[1024*1024*8];

void radar_Callback(const sensor_msgs::PointCloud::ConstPtr& ptcld)
{
 				  if(radar_exist == false)
				 {
					mkdir( (radar_dir).c_str() , 0777);
					radar_exist = true;	
					radar_timetag.open ( "./" +   radar_dir + "_time_tag.txt"); 
				 }
			
			int  radar_ch_cntr = 0;
		   
		   string radar_file_name =  radar_dir + "/" + std::to_string(radarFrameNum) + "_.txt";
		   	  
		   ofstream radar_file (radar_file_name); 
		   
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
			
				 if( radarFrameNum%100 == 0)
					 {
						 cout<<endl;
						 time_t now = time(0);
						tm *ltm = localtime(&now);
						string s;
						char buffer [80];
						strftime(buffer, 80, "%H:%M:%S", ltm);
						(s += buffer) ;
						 cout<<s<<"    "<<radarFrameNum<< "  Frames of radar is processed."<<endl;
					 }
  
}

int radar_bf_FrameNum = 0;
string radar_bf_dir = "Radar_BF";

void radar_bf_Callback(const sensor_msgs::PointCloud::ConstPtr& ptcld)
{
 			   	 if(radarBF_exist == false)
				 {
					mkdir( (radar_bf_dir).c_str() , 0777);
					radarBF_exist = true;		
					radar_bf_timetag.open ( "./" +   radar_bf_dir + "_time_tag.txt"); 
				 }
			  
			  int  radarBF_ch_cntr = 0;
		   
		   string radar_bf_file_name =  radar_bf_dir + "/" + std::to_string(radar_bf_FrameNum) + "_.txt";
		   	   
		   ofstream radar_bf_file (radar_bf_file_name); 
		   
		  
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
bool shutdown = false;

     ros::Subscriber sub_cam1;
	 ros::Subscriber sub_cam2 ;
	 ros::Subscriber sub_cam3 ;
	 ros::Subscriber sub_radar ;
	 ros::Subscriber sub_bf_radar;
	 ros::Subscriber sub_lidar ;

void callback_end(const ros::TimerEvent&)
{	
  shutdown = true;
  sub_cam1.shutdown();
   sub_cam2.shutdown();
    sub_cam3.shutdown();
	sub_radar.shutdown();
	sub_bf_radar.shutdown();
	sub_lidar.shutdown();
	
	
	 cout<<endl;
						 time_t now = time(0);
						tm *ltm = localtime(&now);
						string s;
						char buffer [80];
						strftime(buffer, 80, "%H%M%S", ltm);
						(s += buffer) ;
						 cout<<s<<"  ----   "<<radarFrameNum<< "  Frames of radar is processed."<<endl;
  cout<< " Processing time is almost over."<< endl;
}



int main (int argc, char **argv) 
{ 
	string detected_mode;
	string bag_file;
	
 	vector<string> ModeSreach = { "64N","64F","128N","128F","256N","256F"};
	
	if(argc > 1)
	{
		bag_file =  string(argv[1]);	
	}
	
	   for(int j = 0; j< ModeSreach.size();j++)
	   {
		   if (bag_file.find(ModeSreach[j]) != std::string::npos) 
		   {
					detected_mode = ModeSreach[j];
					break;
			}
	   }
    ///// ROS Init ///////////////////////////////////////////////////////////////////  
    ros::init(argc, argv, "all_sub"); // node name: 
	 
    auto nh = ros::NodeHandle();
    
    //ros::Rate rate(20); // frame rate
	 cout<<endl;
						 time_t now = time(0);
						tm *ltm = localtime(&now);
						string s;
						char buffer [80];
						strftime(buffer, 80, "%H:%M:%S", ltm);
						(s += buffer) ;
						 cout<<s<<"    "<< "  Begining of the session ."<<endl;
	
	
	 sub_cam1 = nh.subscribe("/camera/cam_1", 1000, Cam1_Callback);
	  sub_cam2 = nh.subscribe("/camera/cam_2", 1000, Cam2_Callback);
	  sub_cam3 = nh.subscribe("/camera/cam_3", 1000, Cam3_Callback);
	 sub_radar = nh.subscribe("/radar", 1000, radar_Callback);
	 sub_bf_radar = nh.subscribe("/radar_bf", 1000, radar_bf_Callback);
	 sub_lidar = nh.subscribe("/pandar40p", 1000, lidar_Callback);
	 
    ////////////////////// Radar Start Block ////////////////////////////////////
	
  float inpf;
  if(argc> 2)
 {		 
      string inp(argv[2]);
	 // cout << inp<< endl;
	  inpf = stof(inp);
	 // cout << inpf<< endl;
	  inpf = (inpf *1.15);
 }

     cout<< " The App will record for " << inpf<<" seconds."<<endl;
	 
      ros::Timer timer1 = nh.createTimer(ros::Duration(inpf), callback_end);
	
		ros::Rate r(300); 
		
		while (ros::ok() && (shutdown == false))
		{		
		  ros::spinOnce();
		  r.sleep();
		}
		
		  ofstream radar_metadata (bag_file + "_meta_data.txt"); 
		  
		 radar_metadata<< "Mode" << " = " <<  detected_mode << endl;
		   
		   radar_metadata.close();
		
		
		

 if(shutdown == true)
 {
	 cout<< " Sutting down ROS"<< endl;
	ros::shutdown();
 }
 
  if(radar_exist)
  {
	radar_timetag.close(); 
  }
  
  if(radarBF_exist)
  {
	radar_bf_timetag.close(); 
  }
 
  if( lidar_exist )
  {
		lidar_timetag.close(); 
  }
  
   if( cam1_exist )
   {
		cam1_timetag.close(); 
   }
  if( cam2_exist )
  {
		cam2_timetag.close(); 
  }
  if( cam3_exist )
  {
		cam3_timetag.close(); 
  }
  
  int min = imageFrameNum1;
  
  if(min < imageFrameNum2)
	  min = imageFrameNum2;
  
 if(min < imageFrameNum3)
	  min = imageFrameNum3;
  
  min = min-1;
  
  if(min > 0)
  {
	  cout<< " Concatanating " << min << " images"<< endl;
	 
	  mkdir( (cam_dir).c_str() , 0777);
	  for(int hh=0 ; hh<min; hh++)
	  {
		  Concat(hh) ;
	  }
  }
 
 cout<< " shutting down"<< endl;
  //ros::spin();

  return 0;
	
}
