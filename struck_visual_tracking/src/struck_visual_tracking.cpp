#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

#include "ros/console.h"

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

void writeOutput(const Tracker& tracker, const TrackerInit& tracker_init, std::ofstream& outFile) {
	if (outFile)
	{
		const FloatRect& bb = tracker.GetBB();
		outFile << bb.XMin()/tracker_init.scaleW << "," << bb.YMin()/tracker_init.scaleH << "," << bb.Width()/tracker_init.scaleW << "," << bb.Height()/tracker_init.scaleH << std::endl;
	}
}

bool showOutput(const Config& conf, const cv::Mat& result, TrackerInit& tracker_init, int frameInd, bool paused) {
		if (!conf.quietMode)
		{
			imshow("result", result);
			int key = cv::waitKey(paused ? 0 : 1);
			if (key != -1)
			{
				if (key == 27 || key == 113) // esc q
				{
					return false;
				}
				else if (key == 112) // p
				{
					paused = !paused;
				}
				else if (key == 105 && tracker_init.useCamera)
				{
					tracker_init.doInitialise = true;
				}
			}
			if (conf.debugMode && frameInd == tracker_init.endFrame)
			{
				ROS_INFO_STREAM("\n\nend of sequence, press any key to exit");
				cv::waitKey();
			}
		}
		return true;
}

bool runTracker(const Config& conf, TrackerInit& tracker_init, Tracker& tracker) {
  std::ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), std::ios::out);
		if (!outFile)
		{
			ROS_ERROR_STREAM("error: could not open results file: " << conf.resultsPath);
			return false;
		}
	}

	if (!conf.quietMode)
	{
		cv::namedWindow("result");
	}
	
	cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	auto paused = false;
	srand(conf.seed);
	for (int frameInd = tracker_init.startFrame; frameInd <= tracker_init.endFrame; ++frameInd)
	{
		cv::Mat frame;
		if (!prepareTrackingFrame(conf, tracker_init, tracker, frame, result, frameInd)) {
			return false;
		}
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));

      writeOutput(tracker, tracker_init, outFile);
		}

		if (!showOutput(conf, result, tracker_init, frameInd, paused)) {
			break;
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}

	return true;
}

bool prepareTrackingFrame(const Config& conf, TrackerInit& tracker_init, Tracker& tracker, cv::Mat& frame, cv::Mat& result, int frameInd) {
	if (tracker_init.useCamera) {
			cv::Mat frameOrig;
			tracker_init.cap >> frameOrig;
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			flip(frame, frame, 1);
			frame.copyTo(result);
			if (tracker_init.doInitialise)
			{
				if (tracker.IsInitialised())
				{
					tracker.Reset();
				}
				else
				{
					tracker.Initialise(frame, tracker_init.initBB);
				}
				tracker_init.doInitialise = false;
			}
			else if (!tracker.IsInitialised())
			{
				rectangle(result, tracker_init.initBB, CV_RGB(255, 255, 255));
			}
		}
		else
		{	
			char imgPath[256];
			sprintf(imgPath, tracker_init.imgFormat.c_str(), frameInd);
			cv::Mat frameOrig = cv::imread(imgPath, 0);
			if (frameOrig.empty())
			{
				ROS_ERROR_STREAM("error: could not read frame: " << imgPath);
				return false;
			}
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			cvtColor(frame, result, CV_GRAY2RGB);
		
			if (frameInd == tracker_init.startFrame)
			{
				tracker.Initialise(frame, tracker_init.initBB);
			}
		}
		return true;
}

TrackerInit buildTrackerInit(const Config& conf, const std::string& camera_topic) {
	auto tracker_init = TrackerInit();
	tracker_init.camera_topic = camera_topic;

  // if no sequence specified then use the camera
	tracker_init.useCamera = conf.sequenceName.empty();
	
	if (tracker_init.useCamera)
	{
		if (!tracker_init.cap.open(0))
		{
			ROS_ERROR_STREAM("error: could not start camera capture");
			return tracker_init;
		}
		tracker_init.startFrame = 0;
		tracker_init.endFrame = INT_MAX;
		cv::Mat tmp;
		tracker_init.cap >> tmp;
		tracker_init.scaleW = static_cast<float>(conf.frameWidth/tmp.cols);
		tracker_init.scaleH = static_cast<float>(conf.frameHeight/tmp.rows);

		tracker_init.initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
		ROS_INFO_STREAM("press 'i' to initialise tracker");
	}
	else
	{
		// parse frames file
		const auto framesFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_frames.txt";
		std::ifstream framesFile(framesFilePath.c_str(), std::ios::in);
		if (!framesFile)
		{
			ROS_ERROR_STREAM("error: could not open sequence frames file: " << framesFilePath);
			return tracker_init;
		}
		std::string framesLine;
		getline(framesFile, framesLine);
		sscanf(framesLine.c_str(), "%d,%d", &tracker_init.startFrame, &tracker_init.endFrame);
		if (framesFile.fail() || tracker_init.startFrame == -1 || tracker_init.endFrame == -1)
		{
			ROS_ERROR_STREAM("error: could not parse sequence frames file");
			return tracker_init;
		}
		
		tracker_init.imgFormat = conf.sequenceBasePath+"/"+conf.sequenceName+"/imgs/img%05d.png";
		
		// read first frame to get size
		char imgPath[256];
		sprintf(imgPath, tracker_init.imgFormat.c_str(), tracker_init.startFrame);
		cv::Mat tmp = cv::imread(imgPath, 0);
		tracker_init.scaleW = static_cast<float>(conf.frameWidth/tmp.cols);
		tracker_init.scaleH = static_cast<float>(conf.frameHeight/tmp.rows);
		
		// read init box from ground truth file
		const auto gtFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_gt.txt";
		std::ifstream gtFile(gtFilePath.c_str(), std::ios::in);
		if (!gtFile)
		{
			ROS_ERROR_STREAM("error: could not open sequence gt file: " << gtFilePath);
			return tracker_init;
		}
		std::string gtLine;
		getline(gtFile, gtLine);
		auto xmin = -1.f;
		auto ymin = -1.f;
		auto width = -1.f;
		auto height = -1.f;
		sscanf(gtLine.c_str(), "%f,%f,%f,%f", &xmin, &ymin, &width, &height);
		if (gtFile.fail() || xmin < 0.f || ymin < 0.f || width < 0.f || height < 0.f)
		{
		  ROS_ERROR_STREAM("error: could not parse sequence gt file");
			return tracker_init;
		}
		tracker_init.initBB = FloatRect(xmin*tracker_init.scaleW, ymin*tracker_init.scaleH, width*tracker_init.scaleW, height*tracker_init.scaleH);
	}

	// Initialized a valid object.
	tracker_init.valid = true;
	return tracker_init;
}

bool TrackFromSequence(const std::string& sequence_base_path) {
  return true;
}

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()), rColour);
}
