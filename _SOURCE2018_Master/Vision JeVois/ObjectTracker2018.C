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
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 S- USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#include <jevois/Core/Module.H>
#include <jevois/Debug/Log.H>
#include <jevois/Util/Utils.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Debug/Timer.H>
#include <jevois/Util/Coordinates.H>
#include <jevoisbase/Components/RoadFinder/RoadFinder.H>
#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <jevois/Debug/Profiler.H>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h> // for cvFitLine
#include <string.h>
#include <future>
#include <ctime> // for getting time
#include <cstdio> // for std::remove

static jevois::ParameterCategory const ParamCateg("ObjectTracker2018 Options");

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(hrange, jevois::Range<unsigned char>, "Range of H values for Cubes",
                         jevois::Range<unsigned char>(33, 60), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(srange, jevois::Range<unsigned char>, "Range of S values for Cubes",
                         jevois::Range<unsigned char>(114, 195), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(vrange, jevois::Range<unsigned char>, "Range of V values for Cubes",
                         jevois::Range<unsigned char>(136, 255), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(vrangePlat, jevois::Range<unsigned char>, "Range of V values for platforms",
                         jevois::Range<unsigned char>(200, 255), ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(rrangePlat, jevois::Range<unsigned char>, "Range of ratios for platform width to height",
                         jevois::Range<unsigned char>(10, 20), ParamCateg);

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

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(baseX, size_t, "X coordinate to base objects off of", 
						 160, ParamCateg);

//! Parameter \relates ObjectTracker
JEVOIS_DECLARE_PARAMETER(baseY, size_t, "Y coordinate to base objects off of", 
						 120, ParamCateg);
						 
//! Parameter \relates RoadNavigation
JEVOIS_DECLARE_PARAMETER(vpconf, float, "Minimum vanishing point confidence required to send a serial message",
                         0.0F, roadfinder::ParamCateg);

JEVOIS_DECLARE_PARAMETER(updateTime, double, "Amount of time in ms to send serial messages",
                         50, ParamCateg);

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
                      public jevois::Parameter<hrange, srange, vrange, vrangePlat, rrangePlat, maxnumobj, objectarea, erodesize,
                                               dilatesize, debug, baseX, baseY, vpconf, updateTime>
{	

  public:
     using jevois::StdModule::StdModule;
	 unsigned int missedFrames;
	 double updateTime = updateTime::get();
	 double previousClock = 0;
	 bool resetClock = true;
	 bool roadnav = true;
	 bool trackcube = true;
	 bool trackplat = true;
	 jevois::Timer itsProcessingTimer;
     std::shared_ptr<RoadFinder> itsRoadFinder;
	 //Constuctor init road nav objects
	 ObjectTracker2018(std::string const & instance) :
        jevois::StdModule(instance), itsProcessingTimer("Processing", 30, LOG_DEBUG)
    {
      itsRoadFinder = addSubComponent<RoadFinder>("roadfinder");
	  missedFrames = 0;
    }

    //! Virtual destructor for safe inheritance
    virtual ~ObjectTracker2018() { }
	
	//Put all params into a struct to pass to references
	struct parameters {
	   jevois::Range<unsigned char> hrange, srange, vrange, vrangePlat, rrangePlat;
	   jevois::Range<unsigned int>  objectarea;
	   size_t maxnumobj, erodesize, dilatesize, baseX, baseY;
	   bool debug;
	   float vpconf;
	};
	
    void parseSerial(std::string const & str, std::shared_ptr<jevois::UserInterface> s) override
    {
      std::vector<std::string> tok = jevois::split(str);
      if (tok.empty()) throw std::runtime_error("Unsupported empty module command");
	  
	  if(tok[0] == "cam0")
	  {
	  	 if(tok[1] == "roadnav")
		 {
			roadnav = tok[2] == "true" ? true : false;
		 }
		 else if(tok[1] == "cubetrack")
		 {
			trackcube = tok[2] == "true" ? true : false;
	 	 }
		 else if(tok[1] == "plattrack")
		 {
		    trackplat = tok[2] == "true" ? true : false;
	 	 }
	  }
	  else throw std::runtime_error("Unsupported module command [" + str + ']');
	  
	}
	
	/**
	  * Tracks cubes with a color filter then restricting object ratio
	  * It only returns the closest object
	  */
	std::string trackCube(jevois::RawImage inimg, jevois::RawImage outimg, unsigned int const w, unsigned int const h, parameters params, unsigned int missedFrames)
	{ 
	  // Initialize output
	  std::string output = "";
	  
	  // Exit method if not specified to run
	  if(!trackcube) return output;
	  
	  // Convert input image to BGR24, then to HSV:
      cv::Mat imgbgr = jevois::rawimage::convertToCvBGR(inimg);
      cv::Mat imghsv; 
	  cv::cvtColor(imgbgr, imghsv, cv::COLOR_BGR2HSV);
	  
	  // Threshold the HSV image to only keep pixels within the desired HSV range:
      cv::Mat imgth;
      cv::inRange(imghsv, cv::Scalar(params.hrange.min(), params.srange.min(), params.vrange.min()),
                  cv::Scalar(params.hrange.max(), params.srange.max(), params.vrange.max()), imgth);
      
      // Apply morphological operations to cleanup the image noise:
      cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(params.erodesize, params.erodesize));
      cv::erode(imgth, imgth, erodeElement);
      cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, cv::Size(params.dilatesize, params.dilatesize));
      cv::dilate(imgth, imgth, dilateElement);

      // Detect objects by finding contours:
      std::vector<std::vector<cv::Point> > contours; 
	  std::vector<cv::Vec4i> hierarchy;
      cv::findContours(imgth, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

      // If desired, draw all contours in a thread:
      std::future<void> draw_fut;
      if (params.debug)
        draw_fut = std::async(std::launch::async, [&]() {
            // We reinterpret the top portion of our YUYV output image as an opencv 8UC2 image:
            cv::Mat outuc2(imgth.rows, imgth.cols, CV_8UC2, outimg.pixelsw<unsigned char>()); // pixel data shared
            for (size_t i = 0; i < contours.size(); ++i)
              cv::drawContours(outuc2, contours, i, jevois::yuyv::LightPink, 2, 8, hierarchy);
      });
      
      //Reset vars
      int numobj = 0;
	  int closestObjX = 0;
	  int closestObjY = 0;
	  int largestArea = 0;
	  double closestObjWidth = 0;
	  double closestObjHeight = 0;
	  double closestObjRatio = 0;
	  // Identify the "good" objects:
      if (hierarchy.size() > 0 && hierarchy.size() <= params.maxnumobj)
      {
	    //Reinit vars
        double refArea = 0.0; int x = 0, y = 0; int refIdx = 0;

        //Loop through all detected objects
        for (int index = 0; index >= 0; index = hierarchy[index][0])
        {
          cv::Moments moment = cv::moments((cv::Mat)contours[index]);
          double area = moment.m00;
		  
		  //Check if area is within range
          if (params.objectarea.contains(int(area + 0.4999)) && area > refArea)
          {
		     //Retrieve x and y values
		     x = moment.m10 / area + 0.4999;
		     y = moment.m01 / area + 0.4999;
		     refArea = area;
			 refIdx = index;
			 
			 //Create bounding rects
			 cv::Rect r = cv::boundingRect(contours[index]);
		     double rwidth = r.br().x - r.tl().x;
		     double rheight = r.br().y - r.tl().y;
		     double boxRatio = rwidth/rheight;
		     
			 //Enforce bounding box ratio and only take largest box
			 if(area > largestArea && boxRatio > 0.75 && boxRatio < 1.5 && x != 0 && y != 0)
			 {
			     numobj++;
			     largestArea = area;
				 closestObjX = x;
				 closestObjY = y;
				 closestObjWidth = rwidth;
				 closestObjHeight = rheight;
				 closestObjRatio = boxRatio;
			 }
		  }
        }
		
		// If there is more than one object
        if(numobj >= 1)
		{
		    missedFrames = 0; // Reset missed frame count
		    numobj = 1; // Set to one because we only take largest value
			signed int refX = params.baseX;  // refX and refY need to be assigned to become signed for calculations
			signed int refY = params.baseY;
			
			unsigned int closestObjDist = ((closestObjRatio > .9 ? 15 : 18.5) * w)/closestObjHeight; // Calculation for the distance of the object
			
			jevois::rawimage::drawCircle(outimg, closestObjX, closestObjY, 15, 1, jevois::yuyv::MedPurple); // Draw circle around detected obj
			
		   // Send serial messages to be parsed later
		   output =      "CUBE: w = " + std::to_string(closestObjWidth) +
		   					  " h = " + std::to_string(closestObjHeight) +
							  " r = " + std::to_string(closestObjRatio) +
							  " x = " + std::to_string(closestObjX - refX) +
							  " y = " + std::to_string(closestObjY - refY) +
							  " d = " + std::to_string(closestObjDist);
	   }
	   
	    // Possibly wait until all contours are drawn, if they had been requested:
        if (draw_fut.valid()) draw_fut.get();
	    
    }
	
	// Draw circle around center of (baseX, baseY)
	  jevois::rawimage::drawCircle(outimg, params.baseX, params.baseY, 5, 1, jevois::yuyv::LightPurple);
	
	  // Show number of detected objects:
      jevois::rawimage::writeText(outimg, "Detected " + std::to_string(numobj) + " objects.",
                                  3, h + 2, jevois::yuyv::White);
	  // Return the output speed							  
	  return output;
	}
	
	/**
	  * Track the vanishing point on the horizion and give
	  * the coordinates from the center
	  */
	std::string roadNav(jevois::RawImage inimg, jevois::RawImage outimg, unsigned int const w, unsigned int const h, parameters params)
	{ 
	   // Init the output string
	   std::string output = "";
	   
	   // Exit method if not specified to run
	  if(!roadnav) return output;

       itsProcessingTimer.start();
      
       // Convert it to gray:
       cv::Mat imggray = jevois::rawimage::convertToCvGray(inimg);

       // Compute the vanishing point. Note: the results will be drawn into inimg, so that we don't have to wait for
       // outimg to be ready. It's ok to modify the input image, its buffer will be sent back to the camera driver for
       // capture once we are done here, and will be overwritten anyway:
       itsRoadFinder->process(imggray, outimg);
 
       // Paste the original image + drawings to the top-left corner of the display:
       unsigned short const txtcol = 0xa0ff;
      
       // Clear out the bottom section of the image:
       jevois::rawimage::drawFilledRect(outimg, 0, h, w, outimg.height-h, jevois::yuyv::Black);

       // Get the vanishing point and send to serial:
       std::pair<Point2D<int>, float> vp = itsRoadFinder->getCurrVanishingPoint();
      
       // Write some extra info about the vp:
	   signed int roadX = vp.first.i;
	   signed int width = w;
       std::ostringstream otxt; otxt << std::fixed << std::setprecision(3);
       otxt << "VP x=" << vp.first.i << " (" << vp.second << ") CTR=" << std::setprecision(1);
       auto cp = itsRoadFinder->getCurrCenterPoint();
       auto tp = itsRoadFinder->getCurrTargetPoint();
       float const tpx = itsRoadFinder->getFilteredTargetX();
       otxt << cp.i << " TGT=" << tp.i << " fTPX=" << tpx;
       jevois::rawimage::writeText(outimg, otxt.str().c_str(), 3, h - 13, jevois::yuyv::White);
	   
	   // Set the vp to be based aroung the center of the screen
	   output = "   RN: " + std::to_string(roadX - (width/2)) + "   ";
	   
	   // Return the serial output string
	   return output;
	}

    /**
	  * Track the LEDs on the platform by detecting intensity
	  * then enforcing an object ratio
	  */
	std::string trackPlatform(jevois::RawImage inimg, jevois::RawImage outimg, unsigned int const w, unsigned int const h, parameters params)
	{ 
	  // Init output string
	  std::string output = "";
	  
	  // Exit method if not specified to run
	  if(!trackplat) return output;
	  
	  // Convert input image to BGR24, then to HSV:
      cv::Mat imgbgr = jevois::rawimage::convertToCvBGR(inimg);
      cv::Mat imghsv; 
	  cv::cvtColor(imgbgr, imghsv, cv::COLOR_BGR2HSV);
	  
	  // Threshold the HSV image to only keep pixels within the desired HSV range:
      cv::Mat imgth;
      cv::inRange(imghsv, cv::Scalar(0, 0, params.vrangePlat.min()),
                  cv::Scalar(255, 255, params.vrangePlat.max()), imgth);
      
      // Apply morphological operations to cleanup the image noise:
      cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(params.erodesize, params.erodesize));
      cv::erode(imgth, imgth, erodeElement);
      cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, cv::Size(params.dilatesize, params.dilatesize));
      cv::dilate(imgth, imgth, dilateElement);

      // Detect objects by finding contours:
      std::vector<std::vector<cv::Point> > contours; 
	  std::vector<cv::Vec4i> hierarchy;
      cv::findContours(imgth, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

      // If desired, draw all contours in a thread:
      std::future<void> draw_fut;
      if (params.debug)
        draw_fut = std::async(std::launch::async, [&]() {
            // We reinterpret the top portion of our YUYV output image as an opencv 8UC2 image:
            cv::Mat outuc2(imgth.rows, imgth.cols, CV_8UC2, outimg.pixelsw<unsigned char>()); // pixel data shared
            for (size_t i = 0; i < contours.size(); ++i)
              cv::drawContours(outuc2, contours, i, jevois::yuyv::LightPink, 2, 8, hierarchy);
      });
	  
	  int numobj = 0;
	  int closestObjX = 0;
	  int closestObjY = 0;
	  int largestArea = 0;
	  double closestObjWidth = 0;
	  double closestObjHeight = 0;
	  double closestObjRatio = 0;
	  if (hierarchy.size() > 0 && hierarchy.size() <= params.maxnumobj)
      {
        double refArea = 0.0; int x = 0, y = 0; int refIdx = 0;

        for (int index = 0; index >= 0; index = hierarchy[index][0])
        {
          cv::Moments moment = cv::moments((cv::Mat)contours[index]);
          double area = moment.m00;
		  
          if (params.objectarea.contains(int(area + 0.4999)) && area > refArea)
          {
		     x = moment.m10 / area + 0.4999;
		     y = moment.m01 / area + 0.4999;
		     refArea = area;
			 refIdx = index;
			 
			 cv::Rect r = cv::boundingRect(contours[index]);
		     double rwidth = r.br().x - r.tl().x;
		     double rheight = r.br().y - r.tl().y;
		     double boxRatio = rwidth/rheight;
		  
			 if(area > largestArea && boxRatio > params.rrangePlat.min() 
			    && boxRatio < params.rrangePlat.max() 
				&& x != 0 && y != 0) //Find good object with largest area
			 {
			     numobj++;
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
			signed int refX = params.baseX;  //refX and refY need to be assigned to become signed for calculations
			signed int refY = params.baseY;
			
			jevois::rawimage::drawCircle(outimg, closestObjX, closestObjY, 15, 1, jevois::yuyv::MedPurple); //draw circle around detected obj
			
		   //Send serial messages to be parsed later
		   output =   "   PLAT: r = " + std::to_string(closestObjRatio) +
							  " x = " + std::to_string(closestObjX - refX) +
							  " y = " + std::to_string(closestObjY - refY);
	   } 
	    // Possibly wait until all contours are drawn, if they had been requested:
        if (draw_fut.valid()) draw_fut.get();
	 }
	 
	 return output;
   }
    

    //! Processing function - with USB output
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
	  std::string serMessage = ""; //Init serial Message
      static jevois::Timer timer("processing");
	  updateTime = updateTime::get();

	   if(resetClock){
	      previousClock = std::clock();
		  resetClock = false;
	   }

	  parameters params;
	
      timer.start();

	  // Wait for next available camera image. Any resolution ok, but require YUYV since we assume it for drawings:
      jevois::RawImage inimg = inframe.get(); 
	  unsigned int const w = inimg.width, h = inimg.height;
      inimg.require("input", w, h, V4L2_PIX_FMT_YUYV);

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

      //Get all params
	  params.hrange = hrange::get();
	  params.vrange = vrange::get();
	  params.srange = srange::get();
	  params.vrangePlat = vrangePlat::get();
	  params.rrangePlat = rrangePlat::get();
	  params.dilatesize = dilatesize::get();
	  params.erodesize = erodesize::get();
	  params.objectarea = objectarea::get();
	  params.maxnumobj = maxnumobj::get();
	  params.baseX = baseX::get();
	  params.baseY = baseY::get();
	  params.debug = debug::get();
	  params.vpconf = vpconf::get();

	  // Let camera know we are done processing the input image:
      inframe.done();

	  // Run method on thread
	  auto trackCubeFut = std::async(&ObjectTracker2018::trackCube, this, inimg, outimg, w, h, params, missedFrames);
	  
	  // Run method on thread
	  auto roadNavFut = std::async(&ObjectTracker2018::roadNav, this, inimg, outimg, w, h, params);
	  
	  // Run method on thread
	  auto trackPlatFut = std::async(&ObjectTracker2018::trackPlatform, this, inimg, outimg, w, h, params);
	
	  if(trackcube) serMessage = roadNavFut.get();
	  if(roadnav) serMessage += trackCubeFut.get();
	  if(trackplat) serMessage += trackPlatFut.get();
	  
	  double elaspedTime = (float(std::clock() - previousClock) / (CLOCKS_PER_SEC)) * 1000;
	  
	  if(elaspedTime > updateTime){
	    // Send serial message if there is a message
	    sendSerial(serMessage);
		resetClock = true;
	  }

	 // Show processing fps:
      std::string const & fpscpu = timer.stop();
      jevois::rawimage::writeText(outimg, fpscpu, 3, h - 23, jevois::yuyv::White);
      
      // Send the output image with our processing results to the host over USB:
      outframe.send();
    }
	
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(ObjectTracker2018);
