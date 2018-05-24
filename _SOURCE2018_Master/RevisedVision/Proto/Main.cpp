#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv2/core.hpp"
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <math.h>

using namespace cv;

const int minWidth = 40;
const int minHeight = 40;
const int threshInc = 5;

const double scaleFactor = 0.5;

const Scalar cubeColor = Scalar(255, 0, 100);
const Scalar centerColor = Scalar(255, 255, 0);

Mat output;
Mat src, src_hsv;
Mat dst, detected_edges;
static Mat mergedImage;

std::vector<Mat> channels;

int srcWidth, srcHeight;
int edgeThresh = 1;
int lowThreshold = 40;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_channels = "Output";
char* window_edge_channels = "HSV Edge Channels";

Mat getEdges(Mat src);
void highlightCube(Mat output);
void drawContours(Mat src,  Mat output);
void updateImage(VideoCapture stream, Mat output);
void threadMethod(VideoCapture stream);

int main(int argc, char** argv)
{
	//Get the source image from computer file system
	//src = imread( "../data/cube4.jpg" );

	VideoCapture stream1(0);
	stream1.read(src);

	//Get sizes of image
	srcWidth = src.cols;
	srcHeight = src.rows;

	output = Mat(Size(srcWidth * 2 * scaleFactor, srcHeight * 2 * scaleFactor), CV_8UC3);

	//Create windows
	//namedWindow(window_edge_channels, CV_WINDOW_AUTOSIZE);
	namedWindow(window_channels, CV_WINDOW_AUTOSIZE);
	//namedWindow("1", CV_WINDOW_AUTOSIZE);
	//namedWindow("2", CV_WINDOW_AUTOSIZE);

	dst.create(src.size(), src.type());

	updateImage(stream1, output);

	//std::thread thread_object(threadMethod, stream1);

	while(true)
	{
	waitKey(1);
	output = Scalar::all(0);
	updateImage(stream1, output);
	}

	return 0;
}

void highlightCube(Mat output)
{
	//Add the edges of the second two channels
	Mat image;
	Mat displayImage;
	Mat image0 = getEdges(channels[1]);
	Mat image1 = getEdges(channels[2]);
	addWeighted(image0, 0.5, image1, 0.5, 0.0, image);

	cvtColor(image, displayImage, CV_GRAY2RGB);

	resize(displayImage, displayImage, Size(), scaleFactor * 2, scaleFactor * 2);

	displayImage.copyTo(output(Rect(0, srcHeight * scaleFactor, srcWidth * scaleFactor, srcHeight * scaleFactor)));

	//Show the combined edges
	//imshow("2", image);

	//Draw contours over edges
	drawContours(image, output);
}

Mat getEdges(Mat src)
{
	//Apply guassian filter to remove noise
	blur(src, detected_edges, Size(3, 3));

	//Apply Canny filter to highlight edges
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	
	//Create a black output i mage
	dst = Scalar::all(0);

	//Copy detected edges to the dst Mat
	detected_edges.copyTo(dst);
	return dst;
}

void drawContours(Mat src, Mat output)
{
	Mat canny_output;
	Rect bounding_rect;
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;

	//Return contours
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Mat drawing = Mat::zeros(src.size(), CV_8UC3);
	double largestDimenSum = 0;
	int largestIndex = -1;
	int x, y;

	std::vector<Point> approx;
	//Get largeset contour
	for (int i = 0; i < contours.size(); i++)
	{
		double width = boundingRect(contours[i]).width;
		double height = boundingRect(contours[i]).height;
		double dimenSum = width + height;

		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.035, true);

		//Only use this contour if it is the largest
		if (dimenSum > largestDimenSum && width > minWidth 
			&& height > minHeight && approx.size() == 4 
			&& isContourConvex(Mat(approx)))
		{
			//Calcualte the width to height ratio of the rotated rectangle
			//RotatedRect boundingRectRot = minAreaRect(contours[i]);
			//float angle = boundingRectRot.angle;
			//int heightRot = width / (2 * cos(angle));
			//int widthRot = width / (2 * sin(angle));
			//double rectRatio = heightRot / widthRot;

			//if (0.85 <= rectRatio && rectRatio <= 1.15)
			//{
				largestDimenSum = dimenSum;
				largestIndex = i;
			//}


		}
	}

	//If there is a contour, draw it
	if (largestIndex != -1)
	{
		drawContours(drawing, contours, largestIndex, cubeColor, 2, 8, hierarchy, 0, Point());
		bounding_rect = boundingRect(contours[largestIndex]);

		x = (bounding_rect.width / 2) + bounding_rect.x;
		y = ((bounding_rect.height) / 2) + bounding_rect.y;

		circle(drawing, Point(x, y), 10, centerColor);
	}
	//If there is no contour, lower the threshold
	else
	{
		//lowThreshold = lowThreshold - threshInc > 0 ? lowThreshold - threshInc : lowThreshold;
		//if (lowThreshold - threshInc > 0)
		//	highlightCube(output);
		return;
	}

	resize(drawing, drawing, Size() ,scaleFactor * 2, scaleFactor * 2);
	drawing.copyTo(output(Rect(srcWidth * scaleFactor, srcHeight * scaleFactor, srcWidth * scaleFactor, srcHeight * scaleFactor)));
	cvtColor(output, output, CV_Lab2BGR);
	//resize(output, output, Size(), scaleFactor, scaleFactor);
	imshow(window_channels, output);
	//imshow(window_edge_channels, drawing);
}

void updateImage(VideoCapture stream, Mat output)
{
	stream.read(src);
	
	resize(src, src, Size(), scaleFactor, scaleFactor);

	src.copyTo(output(Rect(0, 0, srcWidth * scaleFactor, srcHeight * scaleFactor)));

	//Create an image to hold each color channel
	Mat channel_mat(Size(srcWidth * scaleFactor, srcHeight * scaleFactor), CV_8UC1);

	//Convert to CIE Lab color space
	cvtColor(src, src, CV_BGR2Lab);

	//Split channels and display them on channel mat
	split(src, channels);
	//channels[0].copyTo(channel_mat(Rect(srcWidth * scaleFactor * 0.5, 0, srcWidth * scaleFactor * 0.5, srcHeight * scaleFactor * 0.5)));
	//channels[1].copyTo(channel_mat(Rect(0, srcHeight * scaleFactor * 0.5, srcWidth * scaleFactor * 0.5, srcHeight * scaleFactor * 0.5)));
	//channels[2].copyTo(channel_mat(Rect(srcWidth * scaleFactor * 0.5, srcHeight * scaleFactor * 0.5, srcWidth * scaleFactor * 0.5, srcHeight * scaleFactor * 0.5)));

	//Remove the L channel - CAUSE WE DON'T TAKE L's
	channels[0] = 0;
	merge(channels, mergedImage);

	//mergedImage.copyTo(output(Rect(srcWidth, 0, srcWidth, srcHeight)));

	//Resize channels to fit on screen
	//resize(channel_mat, channel_mat, Size(), scaleFactor, scaleFactor);

	//Show images
	//imshow("1", mergedImage);
	//imshow(window_channels, channel_mat);
	imshow(window_channels, output);

	highlightCube(output);
}
