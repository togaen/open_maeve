#include "vot.hpp"
#include "params.h"

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "maeve_automation_core/struck_visual_tracking/Tracker.h"
#include "maeve_automation_core/struck_visual_tracking/Config.h"

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()), rColour);
}

int main(int argc, char* argv[]) {
	const auto node_name = std::string("struck_visual_tracking_node");

	// Initialize ROS stuff.
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh(node_name);

	// Load parameters.
  auto params = StruckVisualTrackingParams();
	if (!params.load(nh)) {
		ROS_ERROR_STREAM("Failed to load parameters. Aborting.");
		return EXIT_FAILURE;
	}

	// Debug params?
	if (params.debugMode) {	
		ROS_INFO_STREAM("Loaded params:\n" << params);
	}

	// Initialize STRUCK tracker.
	auto conf = params.toStruckConfig();
	Tracker tracker(conf);

	//Check if --challenge was passed as an argument
	bool challengeMode = false;
	for (int i = 1; i < argc; i++) {
		if (strcmp("--challenge", argv[i]) == 0) {
			challengeMode = true;
		}
	}

	if (challengeMode) {
		//load region, images and prepare for output
		cv::Mat frameOrig;
		cv::Mat frame;
		VOT vot_io("region.txt", "images.txt", "output.txt");
		vot_io.getNextImage(frameOrig);
		resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
		cv::Rect initPos = vot_io.getInitRectangle();
		vot_io.outputBoundingBox(initPos);
		float scaleW = (float)conf.frameWidth/frameOrig.cols;
		float scaleH = (float)conf.frameHeight/frameOrig.rows;

		FloatRect initBB_vot = FloatRect(initPos.x*scaleW, initPos.y*scaleH, initPos.width*scaleW, initPos.height*scaleH);
		tracker.Initialise(frame, initBB_vot);

		while (vot_io.getNextImage(frameOrig) == 1){
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			
			tracker.Track(frame);
			const FloatRect& bb = tracker.GetBB();
			float x = bb.XMin()/scaleW;
			float y = bb.YMin()/scaleH;
			float w = bb.Width()/scaleW;
			float h = bb.Height()/scaleH;

			cv::Rect output = cv::Rect(x,y,w,h);

			vot_io.outputBoundingBox(output);
		}

		return EXIT_SUCCESS;
	}
	
	std::ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), std::ios::out);
		if (!outFile)
		{
			ROS_ERROR_STREAM("error: could not open results file: " << conf.resultsPath);
			return EXIT_FAILURE;
		}
	}
	
	// if no sequence specified then use the camera
	bool useCamera = (conf.sequenceName == "");
	
	cv::VideoCapture cap;
	
	int startFrame = -1;
	int endFrame = -1;
	FloatRect initBB;
	std::string imgFormat;
	float scaleW = 1.f;
	float scaleH = 1.f;
	
	if (useCamera)
	{
		if (!cap.open(0))
		{
			ROS_ERROR_STREAM("error: could not start camera capture");
			return EXIT_FAILURE;
		}
		startFrame = 0;
		endFrame = INT_MAX;
		cv::Mat tmp;
		cap >> tmp;
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;

		initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
		ROS_INFO_STREAM("press 'i' to initialise tracker");
	}
	else
	{
		// parse frames file
		std::string framesFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_frames.txt";
		std::ifstream framesFile(framesFilePath.c_str(), std::ios::in);
		if (!framesFile)
		{
			ROS_ERROR_STREAM("error: could not open sequence frames file: " << framesFilePath);
			return EXIT_FAILURE;
		}
		std::string framesLine;
		getline(framesFile, framesLine);
		sscanf(framesLine.c_str(), "%d,%d", &startFrame, &endFrame);
		if (framesFile.fail() || startFrame == -1 || endFrame == -1)
		{
			ROS_ERROR_STREAM("error: could not parse sequence frames file");
			return EXIT_FAILURE;
		}
		
		imgFormat = conf.sequenceBasePath+"/"+conf.sequenceName+"/imgs/img%05d.png";
		
		// read first frame to get size
		char imgPath[256];
		sprintf(imgPath, imgFormat.c_str(), startFrame);
		cv::Mat tmp = cv::imread(imgPath, 0);
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;
		
		// read init box from ground truth file
		std::string gtFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_gt.txt";
		std::ifstream gtFile(gtFilePath.c_str(), std::ios::in);
		if (!gtFile)
		{
			ROS_ERROR_STREAM("error: could not open sequence gt file: " << gtFilePath);
			return EXIT_FAILURE;
		}
		std::string gtLine;
		getline(gtFile, gtLine);
		float xmin = -1.f;
		float ymin = -1.f;
		float width = -1.f;
		float height = -1.f;
		sscanf(gtLine.c_str(), "%f,%f,%f,%f", &xmin, &ymin, &width, &height);
		if (gtFile.fail() || xmin < 0.f || ymin < 0.f || width < 0.f || height < 0.f)
		{
		  ROS_ERROR_STREAM("error: could not parse sequence gt file");
			return EXIT_FAILURE;
		}
		initBB = FloatRect(xmin*scaleW, ymin*scaleH, width*scaleW, height*scaleH);
	}
	
	
	
	if (!conf.quietMode)
	{
		cv::namedWindow("result");
	}
	
	cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	bool paused = false;
	bool doInitialise = false;
	srand(conf.seed);
	for (int frameInd = startFrame; frameInd <= endFrame; ++frameInd)
	{
		cv::Mat frame;
		if (useCamera)
		{
			cv::Mat frameOrig;
			cap >> frameOrig;
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			flip(frame, frame, 1);
			frame.copyTo(result);
			if (doInitialise)
			{
				if (tracker.IsInitialised())
				{
					tracker.Reset();
				}
				else
				{
					tracker.Initialise(frame, initBB);
				}
				doInitialise = false;
			}
			else if (!tracker.IsInitialised())
			{
				rectangle(result, initBB, CV_RGB(255, 255, 255));
			}
		}
		else
		{			
			char imgPath[256];
			sprintf(imgPath, imgFormat.c_str(), frameInd);
			cv::Mat frameOrig = cv::imread(imgPath, 0);
			if (frameOrig.empty())
			{
				ROS_ERROR_STREAM("error: could not read frame: " << imgPath);
				return EXIT_FAILURE;
			}
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			cvtColor(frame, result, CV_GRAY2RGB);
		
			if (frameInd == startFrame)
			{
				tracker.Initialise(frame, initBB);
			}
		}
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
			
			if (outFile)
			{
				const FloatRect& bb = tracker.GetBB();
				outFile << bb.XMin()/scaleW << "," << bb.YMin()/scaleH << "," << bb.Width()/scaleW << "," << bb.Height()/scaleH << std::endl;
			}
		}
		
		if (!conf.quietMode)
		{
			imshow("result", result);
			int key = cv::waitKey(paused ? 0 : 1);
			if (key != -1)
			{
				if (key == 27 || key == 113) // esc q
				{
					break;
				}
				else if (key == 112) // p
				{
					paused = !paused;
				}
				else if (key == 105 && useCamera)
				{
					doInitialise = true;
				}
			}
			if (conf.debugMode && frameInd == endFrame)
			{
				ROS_INFO_STREAM("\n\nend of sequence, press any key to exit");
				cv::waitKey();
			}
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}
