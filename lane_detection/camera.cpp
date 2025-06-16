#include <atomic>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;


//atomic_bool stop = ATOMIC_VAR_INIT(false);

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
	printf("%d, %d\n", frame_width, frame_height);

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

int main() {
    string displayOption, saveOption, filename;
    displayOption = queryDisplayVid(displayOption);
    filename = querySaveVid(saveOption);

    int point[3][2] = { {300,300}, {200,300}, {400,100}};
	
    int row = sizeof(point)/ sizeof(point[0]); 

    if (displayOption == "Y" && saveOption == "N") {
        displayNoSave(point,row);
    }
    if (displayOption == "Y" && saveOption == "Y") {
        displaySave(filename,point,row);
    }

    destroyAllWindows();
    return 0;
}
