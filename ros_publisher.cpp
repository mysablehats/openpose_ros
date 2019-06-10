///wrapper for 1_extract_from_image.cpp example from openpose.
/// this was done in a hurry. I read an image, publish a skeleton pose, this is
// not done very fast and the topics are not properly time-stamped, so beware
// of possible performance issues.

// ------------------------- OpenPose Library Tutorial - Pose - Example 1 - Extract from Image -------------------------
// This first example shows the user how to:
    // 1. Load an image (`filestream` module)
    // 2. Extract the pose of that image (`pose` module)
    // 3. Render the pose on a resized copy of the input image (`pose` module)
    // 4. Display the rendered pose (`gui` module)
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module: for the Array<float> class that the `pose` module needs
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively

// 3rdparty dependencies
// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include <gflags/gflags.h>
// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif
// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>
//ros dependencies
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point32.h> // this has x,y and z. the third component in the op::array is actually a confidence; this will render incorrectly, but I will be able to publish the message
#include <geometry_msgs/Polygon.h>

// See all the available parameter options withe the `--help` flag. E.g. `build/examples/openpose/openpose.bin --help`
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging/Other

//all of these are parameters. some of them are not useful for us, others are very useful.
//TODO: make these DEFINEs rosparameters and restructure code so they can be set before things are initialized.

//TODO: also, we are going to use ros as a logging tool. if none of the inner openpose bits are using it, it is okay to remove the dependecy here.
DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// Producer
DEFINE_string(image_path,               "examples/media/COCO_val2014_000000000192.jpg",     "Process the desired image.");
// OpenPose
DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(model_folder,             "models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(net_resolution,           "-1x368",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
                                                        " background, instead of being rendered into the original image. Related: `part_to_show`,"
                                                        " `alpha_pose`, and `alpha_pose`.");
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");

image_transport::Publisher pub;
ros::Publisher spub;
sensor_msgs::ImagePtr msgi;
cv_bridge::CvImageConstPtr cv_ptr;

std::string the_model_folder = "/openpose-1.2.1/models/";

bool publishing_image = true;

void mycallback(const sensor_msgs::ImageConstPtr& msg){//does nothing
//locks
//boost::lock_guard<>
//gets image
//unlocks
//ROS_INFO("entered mycallback");
try{
  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
}
catch (cv_bridge::Exception& e)
{
  ROS_ERROR("cv_bridge failed. %s",e.what());
}
//ROS_INFO("extiing callaback");

//NO. not gonna lock anything. this is not going to be threaded. this is just gonna be do one frame capture one frame. that is it!
}

int main(int argc, char *argv[])
{
  try{
    // Parsing command line flags
    // gflags::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "myopenposenode");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("image", 1);
    spub = nh.advertise<geometry_msgs::Polygon>("my_topic", 1);
    image_transport::Subscriber sub = it.subscribe("im_in", 1, mycallback);
    // Running openPoseTutorialPose1
    op::log("OpenPose Library Tutorial - Example 1.", op::Priority::High);
    // ------------------------- INITIALIZATION -------------------------
    // Step 1 - Set logging level
        // - 0 will output all the logging messages
        // - 255 will output nothing
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                  __LINE__, __FUNCTION__, __FILE__);
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 3 - Initialize all required classes
    op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
    op::CvMatToOpInput cvMatToOpInput;
    op::CvMatToOpOutput cvMatToOpOutput;
    op::PoseExtractorCaffe poseExtractorCaffe{poseModel, the_model_folder, FLAGS_num_gpu_start};
    op::PoseCpuRenderer poseRenderer{poseModel, (float)FLAGS_render_threshold, !FLAGS_disable_blending,
                                     (float)FLAGS_alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat;
    //op::FrameDisplayer frameDisplayer{"OpenPose Tutorial - Example 1", outputSize};
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();

    // ------------------------- POSE ESTIMATION AND RENDERING -------------------------
    // Step 1 - Read and load image, error if empty (possibly wrong path)
    // Alternative: cv::imread(FLAGS_image_path, CV_LOAD_IMAGE_COLOR);

    //i think the easiest way to wrap around this is to use a lock and multithread this.
  //  cv::Mat cv_ptr->image = op::loadImage(FLAGS_image_path, CV_LOAD_IMAGE_COLOR);

  //I'm preinitializing everything I can for speed's sake.
    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;

    ros::Rate r(30);
    ROS_INFO("Defined everything ready to acquire.");

    while(nh.ok()){
      if (cv_ptr){
          if(cv_ptr->image.empty())
              op::error("Could not open or find the image: " + FLAGS_image_path, __LINE__, __FUNCTION__, __FILE__);
          const op::Point<int> imageSize{cv_ptr->image.cols, cv_ptr->image.rows};
          // Step 2 - Get desired scale sizes
          std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
              = scaleAndSizeExtractor.extract(imageSize);
          // Step 3 - Format input image to OpenPose input and output formats
          const auto netInputArray = cvMatToOpInput.createArray(cv_ptr->image, scaleInputToNetInputs, netInputSizes);
          auto outputArray = cvMatToOpOutput.createArray(cv_ptr->image, scaleInputToOutput, outputResolution);
          // Step 4 - Estimate poseKeypoints
          poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
          // const op::Array<float> poses; //magic??
          // int num_people = poses.getSize(0);
          // ROS_INFO("found %d people", num_people);
          const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints(); //probably an op::Array<float>
          size_t num_people2 = poseKeypoints.getSize(0);
          size_t num_bodyparts = poseKeypoints.getSize(1);
          ROS_INFO("found %d people from keypoints", (int)num_people2);
          ROS_INFO("found %d num_bodyparts", (int)num_bodyparts);
          ///this can be skipped too if we only want to publish the points, but not the image with the overlay.
          //for number of people? no. I am just going to find one.
          size_t person = 0;
          geometry_msgs::Polygon myperson;
          myperson.points.reserve(num_bodyparts);
          geometry_msgs::Point32 part;

          for (size_t bpart = 0; bpart < num_bodyparts*3; bpart+=3)
          {
            part.x = poseKeypoints[bpart];
            part.y = poseKeypoints[bpart+1];
            part.z = poseKeypoints[bpart+2] ; // or 0;
            myperson.points.push_back(part);
          }
          spub.publish(myperson);

          if (publishing_image)
          {
            // Step 5 - Render poseKeypoints
            poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);
            // Step 6 - OpenPose output format to cv::Mat
            auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);
            msgi = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();

            pub.publish(msgi);
          }
        }
        ros::spinOnce();
        r.sleep();
      }
    // ------------------------- SHOWING RESULT AND CLOSING -------------------------
    // Step 1 - Show results
    //frameDisplayer.displayFrame(outputImage, 0); // Alternative: cv::imshow(outputImage) + cv::waitKey(0)
    // Step 2 - Logging information message
    op::log("Example 1 successfully finished.", op::Priority::High);
    // Return successful message
    return 0;
    }
    catch (ros::Exception& e)
    {
      printf("something went wrong: %s\n", e.what());
      return -1;
    }
}
