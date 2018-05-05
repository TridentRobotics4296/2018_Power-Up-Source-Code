#include <jevois/Core/Module.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Debug/Timer.H>
#include <jevois/Util/Coordinates.H>
#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <jevois/Debug/Profiler.H>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <string.h>
#include <vector>

using namespace cv;

static jevois::ParameterCategory const ParamCateg("ObjectTracker2018 Options");


JEVOIS_DECLARE_PARAMETER(lowThreshold, int, "The value for the threshold limit to declare an edge.",
                         40, ParamCateg);

class ObjectTracker2018 :  public jevois::StdModule,
                      public jevois::Parameter<lowThreshold>
{	
	Mat output;
	
	bool noCube;

	//Colors
	const Scalar cubeColor = Scalar(255, 0, 0);
	const Scalar centerColor = Scalar(0, 0, 255);

	//Int vars
	int minWidth = 25;
	int minHeight = 25;
	int threshDec = 5;
	int lowThreshold = 40;
	int ratio = 3;
	int kernelSize = 3;
	int w, h;
	
	//Holds channels
	std::vector<Mat> channels;
	
	public:
	//Default Constructor
	ObjectTracker2018(std::string const & instance) : jevois::StdModule(instance){ }
	
    //! Virtual destructor for safe inheritance
    virtual ~ObjectTracker2018() { }
	
	/**
	  * Returns a Mat object with the edges of the given image.
	  */
	Mat getEdges(Mat src)
	{
		//Apply guassian to image to remove noise
		Mat edges;
		Mat output;
		blur(src, edges, Size(kernelSize, kernelSize));
		
		//Use Canny edge detector
		Canny(edges, edges, lowThreshold, lowThreshold * ratio, kernelSize);
		
		//Copy edges to black background;
		output = Scalar::all(0);
		edges.copyTo(output);
		
		return output;
	}
	
	/**
	  * Returns a value between -1 and 1 based on placement of box.
	  */
	double calculateError(int cubeX)
	{
		double error = -((w/2 - cubeX) / (w/2));
		return error;
	}
	
	/**
	  * Draws the largest contour of an image.
	  */
	std::string drawContours (Mat src)
	{
		std::vector<std::vector<Point>> contours;
		std::vector<Vec4i> hierarchy;
		
		findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		Mat drawing = Mat::zeros(src.size(), CV_8UC3); 
		double largestDimenSum = 0;
		int largestIndex = -1;
		int x = 0, y = 0;
		
		//Loop through all contours
		for(int i = 0; i < contours.size(); i++)
		{
			//Get info about contour size
			double width = boundingRect(contours[i]).width;
			double height = boundingRect(contours[i]).height;
			double dimenSum = width + height;
			
			//If this is the largest contour and it is above the min size
			if(dimenSum > largestDimenSum && width >= minWidth && height >= minHeight)
			{
				largestDimenSum = dimenSum;
				largestIndex = i;
			}
		}
		
		//If there was an object larger than the min size
		if(largestIndex != -1)
		{
			//Draw the contours of this object
			cv::drawContours(drawing, contours, largestIndex, cubeColor, 2, 8, hierarchy, 0, Point());
			Rect boundingCube = boundingRect(contours[largestIndex]);
			
			//Calculate coords of this object
			x = (boundingCube.width / 2) + boundingCube.x;
			y = (boundingCube.height / 2) + boundingCube.y;
			circle(drawing, Point(x, y), 10, centerColor);
			
			noCube = false;
		}
		//If we can decrease the threshold without going below 0
		else if(lowThreshold - threshDec > 0)
		{
		    //Decrease threshold
			//lowThreshold -= threshDec;
			noCube = true;
		}
		//If there is not object and we cannot decrease threshold
		else
		{
			//There is no object
			noCube = true;
		}
		
		drawing.copyTo(output(Rect(0, h/2, w/2, h/2)));
		return std::to_string(calculateError(x));
	}
	
	/**
	  * Finds cube with edge detection on two seperate color channels,
	  * then draws the largest contour and sends error value.
	  */
	std::string trackCube()
	{
		std::string xPos = 0;
		
		//Perform  edge detection on two channels
		Mat edgesC1 = getEdges(channels[1]);
		Mat edgesC2 = getEdges(channels[2]);
		Mat edges, displayEdges;
		addWeighted(edgesC1, 0.5, edgesC2, 0.5, 0.0, edges);
		
		cvtColor(edges, displayEdges, CV_GRAY2RGB);
		displayEdges.copyTo(output(Rect(w/2, h/2, w/2, h/2)));
		
		//Draw the largest contouor
		xPos = drawContours(edges);
		return xPos;
	}
	
    /**
	  * Processing function - with USB output. This
	  * is called every time the camera updates.
	  */
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
	  static jevois::Timer timer("processing");
	  timer.start();

	  Mat labImage, abImage;
	  std::string serMessage = "";
	  
	  //Only set the lowThresh to the parameter if the user is actively trying to change it.
	  lowThreshold = lowThreshold::get() != 40 ? lowThreshold::get() : lowThreshold;

	  // Wait for next available camera image. Any resolution ok, but require YUYV since we assume it for drawings:
      jevois::RawImage inimg = inframe.get(); 
	  h = inimg.height;
	  w = inimg.width;
      inimg.require("input", w, h, V4L2_PIX_FMT_YUYV);
	  
	  output = Mat(Size(w, h), CV_8UC3);

      // While we process it, start a thread to wait for output frame and paste the input image into it:
      jevois::RawImage outimg; // main thread should not use outimg until paste thread is complete
      auto paste_fut = std::async(std::launch::async, [&]() {
          outimg = outframe.get();
          outimg.require("output", w, h, inimg.fmt);
          jevois::rawimage::paste(inimg, outimg, 0, 0);
          jevois::rawimage::writeText(outimg, "Team 4296 Cube Tracker", 3, 3, jevois::yuyv::White);
          jevois::rawimage::drawFilledRect(outimg, 0, h, w, outimg.height-h, 0x8000);
        });

      // Wait for paste to finish up:
      paste_fut.get();

	  // Let camera know we are done processing the input image:
      inframe.done();

	  //Convert image to CIELab and split channels
	  Mat src = jevois::rawimage::convertToCvBGR(inimg);
	  cvtColor(src, labImage, CV_BGR2Lab);
	  split(labImage, channels);
	  
	  //Remove L channel
	  channels[0] = 0;
	  merge(channels, abImage);
	  
	  src.copyTo(output(Rect(0, 0, w/2, h/2)));
	  abImage.copyTo(output(Rect(w/2, h/2, w/2, h/2)));
	
	  serMessage = trackCube();
	  
	  //Only send the message if we have an object
	  if(!noCube)
	  sendSerial(serMessage);

	  // Show processing fps:
      std::string const & fpscpu = timer.stop();
      jevois::rawimage::writeText(outimg, fpscpu, 3, h - 23, jevois::yuyv::White);
      
      // Send the output image with our processing results to the host over USB:
	  cvtColor(output, output, CV_Lab2BGR);
	  outframe.sendCvBGR(output);
      outframe.send();
    }
	
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(ObjectTracker2018);
