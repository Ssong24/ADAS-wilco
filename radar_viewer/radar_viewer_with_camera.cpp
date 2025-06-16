#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <math.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#define	MAX_NUM_RADAR_TARGETS	65535	

#define WIDTH_VIEWER		520		// pixel
#define	HEIGHT_VIEWER		640		// pixel
#define RANGE_X			20		// meter
#define	RANGE_Y			10		// meter	

#define	PI			3.141592
#define	MARGIN_U		20
#define	MARGIN_V		20

typedef struct radar_target
{
	float x;
	float y;
	float vx;
	float vy;
	float rcs;
}radar_target;

int soc;
int read_can_port;


int open_port(const char *port)
{
	struct ifreq ifr;
	struct sockaddr_can addr;
	
	/* open socket */
	soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (soc < 0)
	{
		printf("error in calling socket()\n");
		return -1;
	}

	addr.can_family = AF_CAN;
	strcpy(ifr.ifr_name,port);
	
	if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
	{
		printf("error in calling ioctl();\n");
		return -1;
	}
	
	addr.can_ifindex = ifr.ifr_ifindex;
	
	fcntl(soc, F_SETFL, O_NONBLOCK);
	
	if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		printf("error in calling bing();\n");
		return -1;
	}
	printf("Socket is open\n");
	
	return 0;
}

int close_port()
{
	close(soc);
	printf("Socket is closed\n");
	
	return 0;
}

/*unsigned char extract_output_type(__u8 *data)
{
	return (unsigned char)((data[5] >> 2) & 0x03);
}*/
unsigned int extract_target_ID(__u8 *data)
{
	__u32 value;
	value = 0x0000;

	value = value | (data[1]<<8);
	value = value | data[0];

	return (unsigned int)value;
}


float extract_dist_long(__u8 *data)
{
	__u32 value;
	value = 0x0000;

	value = value | (data[2] << 3);
	value = value | ((data[3] & 0xE0) >>5);

	return ((float)value) * 0.1;
}

float extract_dist_lat(__u8 *data)
{
	__u32 value;

	value = 0x0000;

	value = value | (data[4]<< 2);
	value = value | ((data[5]&0xC0)>>6);
	
	return -(((float)value) * 0.1 - 51.1);
	
}

float extract_vrel_long(__u8 *data)
{
	__u32 value;
	
	value = 0x0000;
	value = value | (data[5] << 6);
	value = value | ((data[6] & 0xFC)>> 2);

	return ((float)value) * 0.02 - 35.0;
}
	
float extract_vrel_lat(__u8 *data)
{
	__u32 value;
	
	value = 0x0000;
	value = value | data[7];

	return ((float)value) * 0.25 - 32.0;
}


//Camera function

string queryDisplayVid(string& displayOption) {
    cout << "Display? (Y/N)" << endl;
    cin >> displayOption;
    return displayOption;
}

string querySaveVid(string& saveOption) {
    string filename;
    string extension = ".avi";
    cout << "Save? (Y/N)" << endl;
    cin >> saveOption;
    if (saveOption == "Y") {
        cout << "Filename:" << endl;
        cin >> filename;
        filename = filename + extension;
        cout << "Saving the video to "
             << filename << endl;
    }
    else if (saveOption == "N") {
        cout << "Not saving the video." << endl;
    }
    return filename;
}

int displayNoSave(int(*point)[2], int row) {

    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cout << "Error opening video stream." << endl;
        return -1;
    }
    else {       
        while(1) {
            Mat frame;
            cap >> frame;
            if (frame.empty())
                break;
	    
	
            for (int i=0;i<row;i++){
		drawMarker(frame, Point2d(point[i][0], point[i][1]) , Scalar(0, 255, 0), MARKER_CROSS, 10, 1, 8);
	    }

            imshow("Frame", frame);
            char c = (char)waitKey(1);
            if (c == 27)
                break ;
        }
        cap.release();
        return 0;
    }
}

int displaySave(string filename, int(*point)[2], int row) {
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cout << "Error opening video stream." << endl;
        return -1;
    }
    else {
        int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

        VideoWriter video(filename ,CV_FOURCC('M','J','P','G'), 10,
                          Size(frame_width,frame_height));
        while(1) {
            Mat frame;
            cap >> frame;
            if (frame.empty())
                break;
	    
            for (int i=0;i<row;i++){
		drawMarker(frame, Point2d(point[i][0], point[i][1]) , Scalar(0, 255, 0), MARKER_CROSS, 10, 1, 8);
	    }

	    
            video.write(frame);
            imshow("Frame", frame);
            char c = (char)waitKey(1);
            if (c == 27)
                break ;
        }
        cap.release();
        video.release();
        return 0;
    }
}


using namespace std;
using namespace cv;

int main(void)
{	
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		cout << "Error opening video stream." << endl;
		return -1;
	}

	int i, j, read_count;

	clock_t start, end, start_total, end_total;
	float millisec = 0.0, ave_time = 0.0, scale_u_viewer, scale_v_viewer;
 	unsigned long microsec = 0;

	struct radar_target radar_targets[MAX_NUM_RADAR_TARGETS];
	struct can_frame frame_rd, frame;
	int recvbytes = 0;
	unsigned int target_id, num_clusters_near, num_clusters_far, num_objects, count = 0;
	char key = 0;
	const char* windowname = "RADAR";
	double dist;
	double degree;
	
	open_port("slcan0");

	// variables for putText()	
	std::stringstream ss;
	int fontFace = 2;
	double fontScale = 0.3;
	
	int margin_u, margin_v;
	margin_u = MARGIN_U;
	margin_v = MARGIN_V;

	scale_u_viewer = (float)(HEIGHT_VIEWER-margin_u) / RANGE_Y;
	scale_v_viewer = (float)(WIDTH_VIEWER-2*margin_v) / RANGE_X;
	
	double h_fov = ((WIDTH_VIEWER-2*margin_u)/2) * tan((30.0/180.0)*PI);
	cv::Mat img_viewer(HEIGHT_VIEWER, WIDTH_VIEWER, CV_8UC3);
	cv::namedWindow(windowname, WINDOW_AUTOSIZE);
	
	img_viewer = Mat::zeros(img_viewer.rows, img_viewer.cols, img_viewer.type());

	while (1) {
		start = clock();

		for (i = 0; i < MAX_NUM_RADAR_TARGETS; i++) {
			radar_targets[i].x = 0.0;
			radar_targets[i].y = 0.0;
			radar_targets[i].vx = 0.0;
			radar_targets[i].vy = 0.0;
			radar_targets[i].rcs = 0.0;
		}
	
		read_can_port = 1;
		while (read_can_port)
		{
			struct timeval timeout = {1, 0};
			fd_set readSet;
			FD_ZERO(&readSet);
			FD_SET(soc, &readSet);
			
			if (select((soc+1), &readSet, NULL, NULL, &timeout) >= 0)
			{
				if (FD_ISSET(soc, &readSet))
				{				
					recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
					if (recvbytes)
					{	
						
						if (frame_rd.can_id == 0x60B) {
							num_objects = frame_rd.data[0];
							count = 0;
						}
						else if (frame_rd.can_id == 0x60C) {
							count++;
							//target_id = 0x0000;
							//target_id = extract_target_ID(frame_rd.data);
							radar_targets[count].x = extract_dist_long(frame_rd.data);
							radar_targets[count].y = extract_dist_lat(frame_rd.data);
							radar_targets[count].vx = extract_vrel_long(frame_rd.data);
							radar_targets[count].vy = extract_vrel_lat(frame_rd.data);
							//radar_targets[target_id].rcs = extract_rcs(frame_rd.data);
							
						}

					
					}
				}
			}
		
			if (count >= MAX_NUM_RADAR_TARGETS) break;
			else if (count >= num_objects) break;

		}
		end = clock();
		microsec = end - start;
		millisec = (float)microsec / 1000.0;
		//printf("E-time to read radar data: %f ms\n", millisec);
		ave_time =ave_time + millisec;

		// radar viewer code
		img_viewer = Mat::zeros(img_viewer.rows, img_viewer.cols, img_viewer.type());

		// column lines
		line(img_viewer, Point2d(margin_u, 0), Point2d(margin_u, HEIGHT_VIEWER-1-margin_v), Scalar(0,255,255), 1, LINE_8, 0);	
		line(img_viewer, Point2d(margin_u+(WIDTH_VIEWER-2*margin_u)/2, 0), Point2d(margin_u+(WIDTH_VIEWER-2*margin_u)/2, HEIGHT_VIEWER-1-margin_v), Scalar(0,255,255), 1, LINE_8, 0);		
		line(img_viewer, Point2d(WIDTH_VIEWER-margin_u, 0), Point2d(WIDTH_VIEWER-margin_u, HEIGHT_VIEWER-1-margin_v), Scalar(0,255,255), 1, LINE_8, 0);
		
		// row lines
		line(img_viewer, Point2d(margin_u, HEIGHT_VIEWER-1-margin_v), Point2d(WIDTH_VIEWER-margin_u, HEIGHT_VIEWER-1-margin_v), Scalar(0,255,255), 1, LINE_8, 0);

		// fov lines
		line(img_viewer, Point2d((WIDTH_VIEWER-2*margin_u)/2+margin_u, HEIGHT_VIEWER-1-margin_v), Point2d(margin_u, HEIGHT_VIEWER-1-margin_v-h_fov), Scalar(0,255,255), 1, LINE_4, 0);		
		line(img_viewer, Point2d((WIDTH_VIEWER-2*margin_u)/2+margin_u, HEIGHT_VIEWER-1-margin_v), Point2d(WIDTH_VIEWER-margin_u, HEIGHT_VIEWER-1-margin_v-h_fov), Scalar(0,255,255), 1, LINE_4, 0);
			
		printf("NumOfObject = %d\n", num_objects);

		for (i=0; i<num_objects;i++) {
                        
                        printf("id=%2d, x=%2.2f, y=%2.2f\n", i, radar_targets[i].y, radar_targets[i].x);
			dist = radar_targets[i].x*radar_targets[i].x + radar_targets[i].y*radar_targets[i].y;
			dist = sqrt(dist);
			degree = atan2(radar_targets[i].x, abs(radar_targets[i].y))*180/PI;
			printf("distance : %2.2f\n",dist);
			if ((radar_targets[i].x<20)&&(radar_targets[i].y>-5)&&(radar_targets[i].y<5)&&(dist > 0) && (degree >= 30)) {
				drawMarker(img_viewer, Point2d((WIDTH_VIEWER-2*margin_u)/2.0+margin_u + scale_u_viewer*radar_targets[i].y, HEIGHT_VIEWER - margin_v - scale_v_viewer*radar_targets[i].x), Scalar(0, 255, 0), MARKER_CROSS, 10, 1, 8);
				ss.str("");
				ss << i;
				ss << " ("<<radar_targets[i].x<<","<<radar_targets[i].y<< ")";

				putText(img_viewer, (std::string)ss.str(), Point2d((WIDTH_VIEWER-2*margin_u)/2.0+margin_u + (scale_u_viewer*radar_targets[i].y), HEIGHT_VIEWER - margin_v - (scale_v_viewer*radar_targets[i].x)), fontFace, fontScale*2, Scalar(0,255,0), 1, 8);
			}

		}
		
		
		
		imshow(windowname, img_viewer);

		Mat frame;
		cap >> frame;
		if (frame.empty())
			break;
		    
		/*
		for (int i=0;i<row;i++){
			drawMarker(frame, Point2d(point[i][0], point[i][1]) , Scalar(0, 255, 0), MARKER_CROSS, 10, 1, 8);
		 }*/

		imshow("Camera", frame);

		key = waitKey(10);
		if (key == 27) break;
	}


	close_port();
        cap.release();
        video.release();

	return 0;
}

