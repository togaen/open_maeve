#include "maeve_automation_core/struck_visual_tracking/Tracker.h"
#include "maeve_automation_core/struck_visual_tracking/Config.h"

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "vot.hpp"

using namespace cv;

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

void rectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}

int main(int argc, char* argv[])
{
	// read config file
	std::string configPath = "config.txt";
	if (argc > 1)
	{
		configPath = argv[1];
	}
	Config conf(configPath);
	std::cout << conf << std::endl;
	
	if (conf.features.size() == 0)
	{
		std::cout << "error: no features specified in config" << std::endl;
		return EXIT_FAILURE;
	}

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
		Mat frameOrig;
		Mat frame;
		VOT vot_io("region.txt", "images.txt", "output.txt");
		vot_io.getNextImage(frameOrig);
		resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
		cv::Rect initPos = vot_io.getInitRectangle();
		vot_io.outputBoundingBox(initPos);
		float scaleW = (float)conf.frameWidth/frameOrig.cols;
		float scaleH = (float)conf.frameHeight/frameOrig.rows;

		FloatRect initBB_vot = FloatRect(initPos.x*scaleW, initPos.y*scaleH, initPos.width*scaleW, initPos.height*scaleH);
		tracker.Initialise(frame, initBB_vot);

		while (vot_io.getNextImage(frameOrig) == 1){
			resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
			
			tracker.Track(frame);
			const FloatRect& bb = tracker.GetBB();
			float x = bb.XMin()/scaleW;
			float y = bb.YMin()/scaleH;
			float w = bb.Width()/scaleW;
			float h = bb.Height()/scaleH;

			cv::Rect output = cv::Rect(x,y,w,h);

			vot_io.outputBoundingBox(output);
		}

		return 0;
	}
	
	std::ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), std::ios::out);
		if (!outFile)
		{
			std::cout << "error: could not open results file: " << conf.resultsPath << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	// if no sequence specified then use the camera
	bool useCamera = (conf.sequenceName == "");
	
	VideoCapture cap;
	
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
			std::cout << "error: could not start camera capture" << std::endl;
			return EXIT_FAILURE;
		}
		startFrame = 0;
		endFrame = INT_MAX;
		Mat tmp;
		cap >> tmp;
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;

		initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
		std::cout << "press 'i' to initialise tracker" << std::endl;
	}
	else
	{
		// parse frames file
		std::string framesFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_frames.txt";
		std::ifstream framesFile(framesFilePath.c_str(), std::ios::in);
		if (!framesFile)
		{
			std::cout << "error: could not open sequence frames file: " << framesFilePath << std::endl;
			return EXIT_FAILURE;
		}
		std::string framesLine;
		getline(framesFile, framesLine);
		sscanf(framesLine.c_str(), "%d,%d", &startFrame, &endFrame);
		if (framesFile.fail() || startFrame == -1 || endFrame == -1)
		{
			std::cout << "error: could not parse sequence frames file" << std::endl;
			return EXIT_FAILURE;
		}
		
		imgFormat = conf.sequenceBasePath+"/"+conf.sequenceName+"/imgs/img%05d.png";
		
		// read first frame to get size
		char imgPath[256];
		sprintf(imgPath, imgFormat.c_str(), startFrame);
		Mat tmp = cv::imread(imgPath, 0);
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;
		
		// read init box from ground truth file
		std::string gtFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_gt.txt";
		std::ifstream gtFile(gtFilePath.c_str(), std::ios::in);
		if (!gtFile)
		{
			std::cout << "error: could not open sequence gt file: " << gtFilePath << std::endl;
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
		  std::cout << "error: could not parse sequence gt file" << std::endl;
			return EXIT_FAILURE;
		}
		initBB = FloatRect(xmin*scaleW, ymin*scaleH, width*scaleW, height*scaleH);
	}
	
	
	
	if (!conf.quietMode)
	{
		namedWindow("result");
	}
	
	Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	bool paused = false;
	bool doInitialise = false;
	srand(conf.seed);
	for (int frameInd = startFrame; frameInd <= endFrame; ++frameInd)
	{
		Mat frame;
		if (useCamera)
		{
			Mat frameOrig;
			cap >> frameOrig;
			resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
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
			Mat frameOrig = cv::imread(imgPath, 0);
			if (frameOrig.empty())
			{
				std::cout << "error: could not read frame: " << imgPath << std::endl;
				return EXIT_FAILURE;
			}
			resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
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
			int key = waitKey(paused ? 0 : 1);
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
				std::cout << "\n\nend of sequence, press any key to exit" << std::endl;
				waitKey();
			}
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}
