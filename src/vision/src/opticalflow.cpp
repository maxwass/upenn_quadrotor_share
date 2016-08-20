#include "opticalflow.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "timer.h"

using namespace cv;

struct timeval t_init, t_now;;

double t, del_t;
double t_prev = 0.;
float mag_vel[255];

int main(int argc, char **argv)
{

    ros::init(argc, argv, "optical_flow");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("optical_flow", 10);
    ros::Rate loop_rate(500);
    
    FILE *file;
    file = fopen("vo_data.txt","w");
    char sprintf_buffer[5000000];
    int sprintf_buffer_loc = 0;
    
    
    
    double output[5];
    
    gettimeofday(&t_init,NULL);
    
    Mat E_,E, R, T, mask;   
    Mat F(3,3,CV_32FC1);
    Mat F_(3,3,CV_32FC1);
    VideoCapture cap(0); // open the default camera
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FPS, 100); //
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    Mat prevImg_color, prevImg_resize, prevImg;
    Mat currImg_color, currImg_resize, currImg, currImg_;
    Mat dispImg;

    double a_row[] = {547.393358/2., 0.000000, 316.185477/2., 0.000000, 547.551620/2., 228.076200/2., 0.000000, 0.000000, 1.000000/2.};
    Mat A(3,3,CV_64FC1, a_row);
    
    //namedWindow("test",1);
    
    cap >> prevImg_color;
    resize(prevImg_color, prevImg_resize, Size() , 0.2, 0.2, INTER_AREA );
    cvtColor(prevImg_resize, prevImg, CV_BGR2GRAY);
    vector<Point2f> prevFeatures, currFeatures;
    
    
    float velx_, vely_;
    float posx = 0.;
    float posy = 0.;
    
    
    int w = prevImg.rows;
    int h = prevImg.cols;
    size_t nbytes = w * h;
    
    featureDetection(prevImg, prevFeatures);
    
    Vec3b mycolor(100,0,0);
    int myradius=3;
    int MIN_NUM_FEAT = 100; 
    currFeatures = prevFeatures;

    Timer VisionTimer;

    //while(ros::ok())
    //while(t<20.)
    while(ros::ok())
    {
	std_msgs::Float64MultiArray msg;
	
	float velx = 0.; 
        float vely = 0.; 
        int cnt = 0;
    
        cap >> currImg_color; 
        
        //cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
        //cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
        //cap.set(CV_CAP_PROP_FPS, 125);
    
        resize(currImg_color, currImg_resize, Size() , 0.2, 0.2, INTER_AREA );
        cvtColor(currImg_resize, currImg_, CV_BGR2GRAY);
        
        equalizeHist( currImg_, currImg );
        
        gettimeofday(&t_now,NULL);
        t = (t_now.tv_sec - t_init.tv_sec) ;
        t += (t_now.tv_usec - t_init.tv_usec)/1000000.;

        del_t=t-t_prev;
        t_prev=t;
        
        //printf("rate: %e\n",1/del_t);
        if (prevFeatures.size() < MIN_NUM_FEAT)	{
  	  featureDetection(prevImg, prevFeatures);
 	}
 	
 //	printf("Num features: %d\n", prevFeatures.size());
 	
        if(prevFeatures.size() !=0 ){
		vector<uchar> status;
		featureTracking(prevImg, currImg, prevFeatures, currFeatures, status);
		
		vector<Point2f> prevFeatures_norm(prevFeatures.size()), currFeatures_norm(prevFeatures.size());
	    
		double fx = A.at<double>(0,0);
		double fy = A.at<double>(1,1);
		double cx = A.at<double>(0,2);
		double cy = A.at<double>(1,2);
		
		//memset(mag_vel,0,256*sizeof(float));

		for(int i=0;i<prevFeatures.size(); i++){
			prevFeatures_norm.at(i).x = (prevFeatures.at(i).x - cx) / fx;
			currFeatures_norm.at(i).x = (currFeatures.at(i).x - cx) / fx;
			prevFeatures_norm.at(i).y = (prevFeatures.at(i).y - cy) / fy;
			currFeatures_norm.at(i).y = (currFeatures.at(i).y - cy) / fy;
		
			velx_ = currFeatures.at(i).x - prevFeatures.at(i).x;
			vely_ = currFeatures.at(i).y - prevFeatures.at(i).y;
		
			mag_vel[i] = sqrt(velx_*velx_ + vely_*vely_);
		
			if (abs(velx_) < 10. && abs(vely_) < 10. && mag_vel[i] < 20.){
				cnt += 1;
				velx += velx_;
				vely += vely_;
			}
		
		
		
		
		}
	
		if (cnt != 0){
		velx /= cnt;
		vely /= cnt;
		posx += velx;
		posy += vely;
		}
		//printf("%eHz, x: %e, y:%e, velx: %e, vely: %e , %d, %d\n", 1/del_t, posx, posy, velx, vely, isnan(velx), isnan(vely));
	}
	
	

        for (int i=0;i<prevFeatures.size();i++){
    circle(currImg_resize,cvPoint(prevFeatures[i].x,prevFeatures[i].y),myradius,CV_RGB(100,0,0),-1,8,0);
    line(currImg_resize, cvPoint(prevFeatures[i].x,prevFeatures[i].y), cvPoint(currFeatures[i].x,currFeatures[i].y), CV_RGB(250,0,0), 1, 8, 0);
    }
        //drawKeypoints(currImg, currFeatures, dispImg, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        //E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.8, 1.0, mask);

        //prevImg = currImg;
        //memcpy(prevImg, currImg, nbytes);
        prevImg = currImg.clone();
        prevFeatures = currFeatures;
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        //imshow("test", currImg_resize);
        VisionTimer.update();

        output[0] = velx;
        output[1] = vely;
        output[2] = posx;
        output[3] = posy;
	//printf("Freq Camera: %f \n", 1/VisionTimer.getDt());
        output[4] = VisionTimer.getDt();

        for (int i = 0;i<5;i++){
 	msg.data.push_back(output[i]);
	}
         
        chatter_pub.publish(msg);
         	
         	
  /*
         if(sprintf_buffer_loc < sizeof(sprintf_buffer))
            {
                int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc,"%e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e  %e %e %e %e %e\n", t, velx, vely, posx, posy,  mag_vel[0],  mag_vel[1],  mag_vel[2],  mag_vel[3],  mag_vel[4],  mag_vel[5],  mag_vel[6],  mag_vel[7],  mag_vel[8],  mag_vel[9],  mag_vel[10],  mag_vel[11],  mag_vel[12],  mag_vel[13],  mag_vel[14],  mag_vel[15],  mag_vel[16],  mag_vel[17],  mag_vel[18],  mag_vel[19],  mag_vel[20],  mag_vel[21],  mag_vel[22],  mag_vel[23],  mag_vel[24],  mag_vel[25],  mag_vel[26],  mag_vel[27],  mag_vel[28],  mag_vel[29],  mag_vel[30],  mag_vel[31],  mag_vel[32],  mag_vel[33],  mag_vel[34],  mag_vel[35],  mag_vel[36],  mag_vel[37],  mag_vel[38],  mag_vel[39]);


                sprintf_buffer_loc+=sprintf_size;
            }
            else if(sprintf_buffer_loc >= sizeof(sprintf_buffer))
            {
                printf("Warning: sprintf_buffer is full! \n");
            }
          
    */     
            
     //   if(waitKey(30) >= 0) break;
    }
    
    printf("Opening text file.....  ");
    printf("Saving buffer to a file.....  ");
    fwrite(sprintf_buffer, 1, sprintf_buffer_loc,file);
    printf("free buffer file.....  ");
    fclose(file);
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}



