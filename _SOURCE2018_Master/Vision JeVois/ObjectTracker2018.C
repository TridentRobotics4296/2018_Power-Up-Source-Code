// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#include <jevois/Core/Module.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Debug/Timer.H>
#include <jevois/Util/Coordinates.H>

#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string.h>


static jevois::ParameterCategory const ParamCateg("ObjectTracker2018 Options");

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(hrange, jevois::Range<unsigned char>, "Range of H values for HSV window",
                         jevois::Range<unsigned char>(10, 245), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(srange, jevois::Range<unsigned char>, "Range of S values for HSV window",
                         jevois::Range<unsigned char>(10, 245), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(vrange, jevois::Range<unsigned char>, "Range of V values for HSV window",
                         jevois::Range<unsigned char>(10, 245), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(maxnumobj, size_t, "Max number of objects to declare a clean image",
                         10, ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(objectarea, jevois::Range<unsigned int>, "Range of object area (in pixels) to track",
                         jevois::Range<unsigned int>(20*20, 100*100), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(erodesize, size_t, "Erosion structuring element size (pixels)",
                         3, ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(dilatesize, size_t, "Dilation structuring element size (pixels)",
                         8, ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(debug, bool, "Show contours of all object candidates if true",
                         true, ParamCateg);

JEVOIS_DECLARE_PARAMETER(baseX, size_t, "X coordinate to base objects off of", 
						 160, ParamCateg);

JEVOIS_DECLARE_PARAMETER(baseY, size_t, "Y coordinate to base objects off of", 
						 160, ParamCateg);
						 

// icon by Catalin Fertu in cinema at flaticon

//! JeVois sample module
/*! This module is provided as an example of how to create a new standalone module.

    JeVois provides helper scripts and files to assist you in programming new modules, following two basic formats:

    - if you wish to only create a single module that will execute a specific function, or a collection of such modules
      where there is no shared code between the modules (i.e., each module does things that do not relate to the other
      modules), use the skeleton provided by this sample module. Here, all the code for the sample module is compiled
      into a single shared object (.so) file that is loaded by the JeVois engine when the corresponding video output
      format is selected by the host computer.

    - if you are planning to write a collection of modules with some shared algorithms among several of the modules, it
      is better to first create machine vision Components that implement the algorithms that are shared among several of
      your modules. You would then compile all your components into a first shared library (.so) file, and then compile
      each module into its own shared object (.so) file that depends on and automatically loads your shared library file
      when it is selected by the host computer. The jevoisbase library and collection of components and modules is an
      example for how to achieve that, where libjevoisbase.so contains code for Saliency, ObjectRecognition, etc
      components that are used in several modules, and each module's .so file contains only the code specific to that
      module.

    @author Sample Author

    @videomapping YUYV 640 480 28.5 YUYV 640 480 28.5 SampleVendor ObjectTracker2018
    @email sampleemail\@samplecompany.com
    @address 123 First Street, Los Angeles, CA 90012
    @copyright Copyright (C) 2017 by Sample Author
    @mainurl http://samplecompany.com
    @supporturl http://samplecompany.com/support
    @otherurl http://samplecompany.com/about
    @license GPL v3
    @distribution Unrestricted
    @restrictions None */
class ObjectTracker2018 :  public jevois::StdModule,
                      public jevois::Parameter<hrange, srange, vrange, maxnumobj, objectarea, erodesize,
                                               dilatesize, debug, baseX, baseY>
{
  public:
    //! Default base class constructor ok
    using jevois::StdModule::StdModule;

    //! Virtual destructor for safe inheritance
    virtual ~ObjectTracker2018() { }

	//private:
	int missedFrames = 0;
	
	std::string trackCube(jevois::RawImage inimg, jevois::RawImage outimg, unsigned int const w, unsigned int const h)
	{ 
	  std::string output = "";
	
	  // Convert input image to BGR24, then to HSV:
      cv::Mat imgbgr = jevois::rawimage::convertToCvBGR(inimg);
      cv::Mat imghsv; cv::cvtColor(imgbgr, imghsv, cv::COLOR_BGR2HSV);
	  
		// Threshold the HSV image to only keep pixels within the desired HSV range:
      cv::Mat imgth;
      cv::inRange(imghsv, cv::Scalar(hrange::get().min(), srange::get().min(), vrange::get().min()),
                  cv::Scalar(hrange::get().max(), srange::get().max(), vrange::get().max()), imgth);
      
      // Apply morphological operations to cleanup the image noise:
      cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(erodesize::get(), erodesize::get()));
      cv::erode(imgth, imgth, erodeElement);

      cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, cv::Size(dilatesize::get(), dilatesize::get()));
      cv::dilate(imgth, imgth, dilateElement);

      // Detect objects by finding contours:
      std::vector<std::vector<cv::Point> > contours; 
	  std::vector<cv::Vec4i> hierarchy;
      cv::findContours(imgth, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

      // If desired, draw all contours in a thread:
      std::future<void> draw_fut;
      if (debug::get())
        draw_fut = std::async(std::launch::async, [&]() {
            // We reinterpret the top portion of our YUYV output image as an opencv 8UC2 image:
            cv::Mat outuc2(imgth.rows, imgth.cols, CV_8UC2, outimg.pixelsw<unsigned char>()); // pixel data shared
            for (size_t i = 0; i < contours.size(); ++i)
              cv::drawContours(outuc2, contours, i, jevois::yuyv::LightPink, 2, 8, hierarchy);
      });
      
      // Identify the "good" objects:
      int numobj = 0;
	  int closestObjX = 0;
	  int closestObjY = 0;
	  int largestArea = 0;
	  double closestObjWidth = 0;
	  double closestObjHeight = 0;
	  double closestObjRatio = 0;
      if (hierarchy.size() > 0 && hierarchy.size() <= maxnumobj::get())
      {
        double refArea = 0.0; int x = 0, y = 0; int refIdx = 0;

        for (int index = 0; index >= 0; index = hierarchy[index][0])
        {
          cv::Moments moment = cv::moments((cv::Mat)contours[index]);
          double area = moment.m00;
		  
          if (objectarea::get().contains(int(area + 0.4999)) && area > refArea)
          {
		  	 numobj++;
		     x = moment.m10 / area + 0.4999;
		     y = moment.m01 / area + 0.4999;
		     refArea = area;
			 refIdx = index;
			 
			 cv::Rect r = cv::boundingRect(contours[index]);
		     double rwidth = r.br().x - r.tl().x;
		     double rheight = r.br().y - r.tl().y;
		     double boxRatio = rwidth/rheight;
		  
			 if(area > largestArea && boxRatio > 0.8 && boxRatio < 1.3) //Find good object with largest area
			 {
			     largestArea = area;
				 closestObjX = x;
				 closestObjY = y;
				 closestObjWidth = rwidth;
				 closestObjHeight = rheight;
				 closestObjRatio = boxRatio;
			 }
		  }
        }
		
        if(numobj >= 1)
		{ //If there is more than one object found
		    missedFrames = 0; //Reset missed frame count
			numobj = 1; //Set to one because we only take largest value
			signed int refX = baseX::get();  //refX and refY need to be assigned to become signed for calculations
			signed int refY = baseY::get();
			
			unsigned int closestObjDist = ((closestObjRatio > .9 ? 15 : 18.5) * 254)/closestObjHeight;
			
			jevois::rawimage::drawCircle(outimg, closestObjX, closestObjY, 15, 1, jevois::yuyv::MedPurple); //draw circle around detected obj
			
		   //Send serial messages to be parsed later
		   output =   "Object: w=" + std::to_string(closestObjWidth) +
		   					  "h=" + std::to_string(closestObjHeight) +
							  "r=" + std::to_string(closestObjRatio) +
							  "x=" + std::to_string(closestObjX - refX) +
							  "y=" + std::to_string(closestObjY - refY) +
							  "d=" + std::to_string(closestObjDist);
	   }
	   else if(missedFrames > 5)
	   {
		    missedFrames++;
			output = "Object: NO OBJECT";
	   }
	   else
	   {
	       missedFrames++;
	   }
	   
	    // Possibly wait until all contours are drawn, if they had been requested:
        if (draw_fut.valid()) draw_fut.get();
	   
	    
    }
	
	  jevois::rawimage::drawCircle(outimg, baseX::get(), baseY::get(), 5, 1, jevois::yuyv::LightPurple); //Draw circle around center of (baseX, baseY)
	
	  // Show number of detected objects:
      jevois::rawimage::writeText(outimg, "Detected " + std::to_string(numobj) + " objects.",
                                  3, h + 2, jevois::yuyv::White);
	  return output;
	}
	
	std::string roadNav(jevois::RawImage inimg, jevois::RawImage outimg, unsigned int const w, unsigned int const h)
	{ 
	   std::string output = "";
	
	  // Convert input image to BGR24, then to HSV:
      cv::Mat imgbgr = jevois::rawimage::convertToCvBGR(inimg);
      cv::Mat imghsv; cv::cvtColor(imgbgr, imghsv, cv::COLOR_BGR2HSV);
	  
		// Threshold the HSV image to only keep pixels within the desired HSV range:
      cv::Mat imgth;
      cv::inRange(imghsv, cv::Scalar(hrange::get().min(), srange::get().min(), vrange::get().min()),
                  cv::Scalar(hrange::get().max(), srange::get().max(), vrange::get().max()), imgth);
				  
	  jevois::rawimage::drawCircle(outimg, 60, 60, 10, 1, jevois::yuyv::MedPurple); //draw circle around detected obj
				  
				  
	   return output;
	}
	

    //! Processing function - with USB output
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
	  std::string serMessage = ""; //Init serial Message
      static jevois::Timer timer("processing");

      timer.start();

	  // Wait for next available camera image. Any resolution ok, but require YUYV since we assume it for drawings:
      jevois::RawImage inimg = inframe.get(); 
	  unsigned int const w = inimg.width, h = inimg.height;
      inimg.require("input", w, h, V4L2_PIX_FMT_YUYV);

      // While we process it, start a thread to wait for output frame and paste the input image into it:
      jevois::RawImage outimg; // main thread should not use outimg until paste thread is complete
      auto paste_fut = std::async(std::launch::async, [&]() {
          outimg = outframe.get();
          outimg.require("output", w, h + 14, inimg.fmt);
          jevois::rawimage::paste(inimg, outimg, 0, 0);
          jevois::rawimage::writeText(outimg, "Team 4296 Cube Tracker", 3, 3, jevois::yuyv::White);
          jevois::rawimage::drawFilledRect(outimg, 0, h, w, outimg.height-h, 0x8000);
        });

      // Wait for paste to finish up:
      paste_fut.get();

	  // Get string output from trackCube method
	  serMessage = trackCube(inimg, outimg, w, h);
	  serMessage += roadNav(inimg, outimg, w, h);
	 
	  // Send serial message if there is a message
	  if(serMessage != "")
	     sendSerial(serMessage);
	 
	  // Let camera know we are done processing the input image:
      inframe.done();

	 // Show processing fps:
      std::string const & fpscpu = timer.stop();
      jevois::rawimage::writeText(outimg, fpscpu, 3, h - 13, jevois::yuyv::White);
      
      // Send the output image with our processing results to the host over USB:
      outframe.send();
    }
	
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(ObjectTracker2018);
