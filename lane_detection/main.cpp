#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "include/LaneDetector.hpp"
/*
 Mat data type:
 CV_32F  image.at<float>(i,j)
 CV_64F image.at<double>(i,j) ªÁøÎ!
*/

#define PI 3.14159265

using namespace cv;

Mat absSobelThresh(Mat image, char orient, int minThresh, int maxThresh);
Mat magThresh(Mat image, int sobel_kernel, int thresh_x, int thresh_y);
Mat dirThresh(Mat image, int sobel_kernel, float thresh_x, float thresh_y);
Mat hlsSelect(Mat image, int minSthresh, int maxSthresh, int minLthresh, int maxLthresh);


int main() {
	Mat original_image;
	Mat image;
	char orient_ch = 'x';

	original_image = imread("input_image/test1.jpg");

	resize(original_image, image, Size(original_image.cols/2, original_image.rows/2), 0, 0, INTER_CUBIC);


	printf("width: %d   height: %d\n", image.cols, image.rows);

	Mat abs_Image;

	Mat mag_image;
	mag_image = magThresh(image, 3, 20, 100);

	Mat dir_image;
	dir_image = dirThresh(image,3, 0.8, 1.2);

	Mat hls_image; 
	hls_image = hlsSelect(image, 140, 255, 120, 255);

}



Mat absSobelThresh(Mat image, char orient, int minThresh, int maxThresh) {
	// 1) Convert to grayscale
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);
	
	// 2) Apply x or y gradient and take the absolute value
	Mat abs_sobel;
	if (orient == 'x') {
		Sobel(gray, abs_sobel, CV_32F, 1, 0);		// c++ø°º± CV_32F æ≤±‚
		abs_sobel = abs(abs_sobel);
	}
	
	if (orient == 'y') {
		Sobel(gray, abs_sobel, CV_32F, 0, 1);		// 32bit float data type => access pixel value by "image.at<float>(i,j)"
		abs_sobel = abs(abs_sobel);
	}
	
	// 3) Rescale back to 8 bit integer
	// 3-1) Find Maximum value in sobel
	float max = 0;
	for (int j = 0; j < abs_sobel.cols; j++) {
		for (int i = 0; i < abs_sobel.rows; i++) {
			if (max < abs_sobel.at<float>(i, j))
				max = abs_sobel.at<float>(i, j);

		}
	}

	// 3-2) Rescale 
	for (int j = 0; j < abs_sobel.cols; j++) {
		for (int i = 0; i < abs_sobel.rows; i++) {
			abs_sobel.at<float>(i,j) = 255.0*abs_sobel.at<float>(i,j)/max;
		}
	}
	
	// 4) Binarize gray image
	for (int j = 0; j < abs_sobel.cols; j++) {
		for (int i = 0; i < abs_sobel.rows; i++) {

			if (abs_sobel.at<float>(i, j) >= minThresh && abs_sobel.at<float>(i, j) <= maxThresh)
				abs_sobel.at<float>(i, j) = 1;
			else
				abs_sobel.at<float>(i, j) = 0;
		}
	}

	return abs_sobel;

}


Mat magThresh(Mat image, int sobel_kernel, int minThresh, int maxThresh)
{
	// 1)Convert to Gray Image
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);

	// 2)Take the gradient in x and y separately
	Mat sobelX;
	Mat sobelY;

	Sobel(gray, sobelX, CV_32F, 1, 0, sobel_kernel);
	Sobel(gray, sobelY, CV_32F, 0, 1, sobel_kernel);

	// 3)Calculate the xy magnitude							 -- ¿⁄∑·«¸ ¬˜¿Ã ª˝∞¢!
	Mat mag;
	Sobel(gray, mag, CV_32F, 1, 0, sobel_kernel);

	for (int j = 0; j < gray.cols; j++) {
		for (int i = 0; i < gray.rows; i++) {
			mag.at<float>(i,j) = sqrt(pow(sobelX.at<float>(i, j), 2) + pow(sobelY.at<float>(i, j), 2));
	
		}
	}

	// 4) Scale to 8-bit (0~255) and convert to type np.uint8
	// find Maximum value in sobel
	float max = 0;
	for (int j = 0; j < mag.cols; j++) {
		for (int i = 0; i < mag.rows; i++) {
			if (mag.at<float>(i, j) > max)
				max = mag.at<float>(i, j);
		}
	}


	// Scale 
	for (int j = 0; j < mag.cols; j++) {
		for (int i = 0; i < mag.rows; i++) {
			mag.at<float>(i, j) = 255.0 * mag.at<float>(i,j) / max;
		}
	}

	// 5) Create a binary mask where mag thresholds are met
	for (int j = 0; j < mag.cols; j++) {
		for (int i = 0; i < mag.rows; i++) {
			//printf("abs_sobel pixel value : %f\n", abs_sobel.at<float>(i, j));
			if (mag.at<float>(i, j) >= minThresh && mag.at<float>(i, j) <= maxThresh)
				mag.at<float>(i, j) = 1;
			else
				mag.at<float>(i, j) = 0;
		}
	}

	return mag;

}

Mat dirThresh(Mat image, int sobel_kernel, float minThresh, float maxThresh) {
	// 1) Conver to grayscale
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);

	// 2) Take the gradient in x and y separately
	Mat sobelX;
	Mat sobelY;

	Sobel(gray, sobelX, CV_32F, 1, 0, sobel_kernel);
	Sobel(gray, sobelY, CV_32F, 0, 1, sobel_kernel);

	// 3) Use arctan2(sobelY, sobelY) to calculate the direction of the gradient
	Mat direction;
	direction.create(sobelX.rows, sobelX.cols, CV_32F);
	for (int j = 0; j < direction.cols; j++) {
		for (int i = 0; i < gray.rows; i++) {
			direction.at<float>(i, j) = atan2(sobelY.at<float>(i, j), sobelX.at<float>(i, j));
		}
	}


	Mat binary_output;
	binary_output.create(direction.rows, direction.cols, CV_32F);
	for (int j = 0; j < direction.cols; j++) {
		for (int i = 0; i < direction.rows; i++) {
			//printf("abs_sobel pixel value : %f\n", abs_sobel.at<float>(i, j));
			if (direction.at<float>(i, j) >= minThresh && direction.at<float>(i, j) <= maxThresh)
				binary_output.at<float>(i, j) = 1;
			else
				binary_output.at<float>(i, j) = 0;
		}
	}

	return binary_output;
}

Mat hlsSelect(Mat image, int minSthresh, int maxSthresh, int minLthresh, int maxLthresh)
{
	// 1)Convert to HLS color space
	Mat hls_img = Mat(image.rows, image.cols, CV_8U);
	cvtColor(image, hls_img, CV_BGR2HLS);
	imshow("HLS image", hls_img);

	// separate into three images with only one channel H,L, and S
	Mat H = Mat(image.rows, image.cols, CV_8U);
	Mat L = Mat(image.rows, image.cols, CV_8U);
	Mat S = Mat(image.rows, image.cols, CV_8U);

	// Invert channels
	// Don't copyy data, just the matrix headers
	std::vector<Mat> hls_channel{ H, L, S };

	// Create the output
	split(hls_img, hls_channel);

	// 3)Return a binary Image of threshold result
	Mat binary_output;
	binary_output.create(image.rows, image.cols, CV_8U);
	
	for (int j = 0; j < image.cols; j++) {
		for (int i = 0; i < image.rows; i++) {
			//printf("abs_sobel pixel value : %f\n", abs_sobel.at<float>(i, j));
			if (S.at<uchar>(i, j) >= minSthresh && S.at<uchar>(i, j) <= maxSthresh && L.at<uchar>(i, j) >= minLthresh && L.at<uchar>(i, j) <= maxLthresh) {
				binary_output.at<uchar>(i, j) = 255;
			}
			else
				binary_output.at<uchar>(i, j) = 0;
		}
	}

	return binary_output;

}

Mat binaryPipeline(Mat image){

	return image;
}

Mat warp_image(Mat image) {
	int x = image.cols;
	int y = image.rows;

	Point2f inputQuad[4];
	Point2f outputQuad[4];

	int source_points[4][2] =
	{ { 0.117 * x, y },
	{ (0.5 * x) - (x*0.078), (2 / 3)*y },
	{ (0.5 * x) + (x*0.078), (2 / 3)*y },{ x - (0.117 * x), y } };

	int destination_points[4][2] = {
		{ 0.25 * x, y },{ 0.25 * x, 0 },{ x - (0.25 * x), 0 },{ x - (0.25 * x), y }
	};


	return image;
}






