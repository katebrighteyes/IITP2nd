// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ros/package.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>




// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <exception>
#include <csignal>

// darknet_ros_msgs
#include "ros_dnn_test/BoundingBoxes.h"
#include "ros_dnn_test/BoundingBox.h"


using namespace cv;
using namespace dnn;
using namespace std;

static const std::string OPENCV_WINDOW = "Yolo_node";

static bool bExit = false;

void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received.\n";

   bExit = true;

   exit(signum);  
}

class DarknetRos
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher bbox_pub_;
  
  string str, outputFile;
  VideoCapture cap;
  VideoWriter video;
  Mat frame, blob; //camImageCopy_

  Net net;
  vector<string> classes;
  String video_path;
  String classesFile;
  String modelConfiguration;
  String modelWeights;
  float confThreshold;  //Confidence threshold
  float nmsThreshold;   //Non-maximum suppression threshold
  int inpWidth;       //Width of network's input image
  int inpHeight;     //Height of network's input image
  int frameWidth;
  int frameHeight;
  bool imageStatus_ = false;
  std_msgs::Header imageHeader;
  std::thread yoloThread_;
  std::thread bboxThread_;

  string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;

  //! Detected objects.
  //vector<vector<RosBox_> > rosBoxes_;
  //vector<int> rosBoxCounter_;
  ros_dnn_test::BoundingBoxes boundingBoxesResults_;
  std_msgs::Header imageHeader_;


public:
  DarknetRos() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    ROS_INFO("ros_node");
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &DarknetRos::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    boundingBoxesTopicName = string("/ros_dnn_test/bounding_boxes");
    boundingBoxesQueueSize = 1;
    boundingBoxesLatch = false;


    //bbox_pub_ = nh.advertise<darknet_sub::MsgState>("cur_state", 100);
    bbox_pub_ = nh_.advertise<ros_dnn_test::BoundingBoxes>( boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);

    cv::namedWindow(OPENCV_WINDOW);

    string pkgpath = ros::package::getPath("ros_dnn_test");

    ROS_INFO("pkgpath:%s", pkgpath.c_str());

    video_path = pkgpath + "/data/run.mp4";
    classesFile = pkgpath + "/data/coco.names";
    modelConfiguration = pkgpath + "/data/yolov3-tiny.cfg";
    modelWeights = pkgpath + "/data/yolov3-tiny.weights";
    ROS_INFO("modelConfiguration:%s", modelConfiguration.c_str());
    ROS_INFO("modelWeights:%s", modelWeights.c_str());

    //ROS_INFO("readNetFromDarknet");
    net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);

    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Initialize the parameters
    confThreshold = 0.3;
    nmsThreshold = 0.4;
    inpWidth = 416;
    inpHeight = 416;
    frameWidth = 640;
    frameHeight = 480;

  }

  ~DarknetRos()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //ROS_INFO("imageCb");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
#if 0	
	Mat frame = cv_ptr->image;
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, frame);
    cv::waitKey(3);

#endif	


#if 1
    Mat frame = cv_ptr->image;
    if (!frame.empty())
    {
      //frame = cam_image->image.clone();
        imageHeader_ = msg->header;
        resize(frame, frame, Size(frameWidth, frameHeight));
        blobFromImage(frame, blob, 1/255.0, cv::Size(inpWidth, inpHeight), Scalar(0,0,0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence
        int outnum = postprocess(frame, outs);
            //yoloThread_ = std::thread(&DarknetRos::yolo, this, outnum);

        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        //putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        // Write the frame with the detection boxes
        Mat detectedFrame;
        frame.convertTo(detectedFrame, CV_8U);

        cv::imshow(OPENCV_WINDOW, detectedFrame);
        cv::waitKey(1);		
        publishBBox(outnum);

    }
#endif
  }



#if 1
//void publishInThread()
void publishBBox(int outnum)
{

  // Publish bounding boxes and detection result.
  if (outnum > 0 && outnum <= 100) {
    boundingBoxesResults_.header.stamp = ros::Time::now();
    boundingBoxesResults_.header.frame_id = "detection";
    boundingBoxesResults_.image_header = imageHeader_;
    bbox_pub_.publish(boundingBoxesResults_);
  } 
  boundingBoxesResults_.bounding_boxes.clear();
}

	// Remove the bounding boxes with low confidence using non-maxima suppression
	int postprocess(Mat& frame, const vector<Mat>& outs)
	{
	    vector<int> classIds;
	    vector<float> confidences;
	    vector<Rect> boxes;
            int outnum = 0;
		
	    
	    for (size_t i = 0; i < outs.size(); ++i)
	    {
		// Scan through all the bounding boxes output from the network and keep only the
		// ones with high confidence scores. Assign the box's class label as the class
		// with the highest score for the box.
		float* data = (float*)outs[i].data;
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
		    Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
		    Point classIdPoint;
		    double confidence;
		    // Get the value and location of the maximum score
		    minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
		    if (confidence > confThreshold)
		    {
		        int centerX = (int)(data[0] * frame.cols);
		        int centerY = (int)(data[1] * frame.rows);
		        int width = (int)(data[2] * frame.cols);
		        int height = (int)(data[3] * frame.rows);
		        int left = centerX - width / 2;
		        int top = centerY - height / 2;
		        
		        classIds.push_back(classIdPoint.x);
		        confidences.push_back((float)confidence);
		        boxes.push_back(Rect(left, top, width, height));
		    }
		}
	    }
	    
	    // Perform non maximum suppression to eliminate redundant overlapping boxes with
	    // lower confidences
	    vector<int> indices;
	    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
            ros_dnn_test::BoundingBox boundingBox;
            outnum = indices.size();
	    for (size_t i = 0; i < indices.size(); ++i)
	    {
		int idx = indices[i];
		Rect box = boxes[idx];
                string clslabel;
	  	drawPred(classIds[idx], confidences[idx], box.x, box.y,
		         box.x + box.width, box.y + box.height, frame, clslabel);
                boundingBox.Class = clslabel;
          	boundingBox.probability = confidences[idx];
          	boundingBox.xmin = box.x;
          	boundingBox.ymin = box.y;
          	boundingBox.xmax = box.x + box.width;
          	boundingBox.ymax = box.y + box.height;
          	boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
	    }
            return outnum;
	}

	// Draw the predicted bounding box
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, string &clslabel)
	{
	    //Draw a rectangle displaying the bounding box
	    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);
	    
	    //Get the label for the class name and its confidence
	    string label = format("%.2f", conf);
	    if (!classes.empty())
	    {
		CV_Assert(classId < (int)classes.size());
                clslabel = classes[classId];
		label = clslabel + ":" + label;
	    }
	    
	    //Display the label at the top of the bounding box
	    int baseLine;
	    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	    top = max(top, labelSize.height);
	    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
	    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
	}

	// Get the names of the output layers
	vector<String> getOutputsNames(const Net& net)
	{
	    static vector<String> names;
	    if (names.empty())
	    {
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		vector<int> outLayers = net.getUnconnectedOutLayers();
		
		//get the names of all the layers in the network
		vector<String> layersNames = net.getLayerNames();
		
		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
		names[i] = layersNames[outLayers[i] - 1];
	    }
	    return names;
	}
#endif
};

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);  
  ros::init(argc, argv, "ros_node");
  ROS_INFO("ros_node");
  DarknetRos ic;
  ros::spin();
  return 0;
}



