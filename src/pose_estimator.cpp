#include "pose_estimator.h"

PoseEstimator::PoseEstimator() {
  ROS_INFO("starting pose estimation");
  // pose_keypoints_0 = pose_keypoints_1 = nullptr;
};

void PoseEstimator::run_openpose(cv::Mat frame, op::Array<float> *pose_keypoints) {

  /// READING GOOGLE FLAGS
  const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
  const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
  const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);

  // checking contradictory flags
  if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
    op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
  if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
    op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1", __LINE__, __FUNCTION__, __FILE__);

  // enable Google logging
  const bool enableGoogleLogging = true;

  /// INITIALIZE REQUIRED CLASSES
  op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
  op::CvMatToOpInput cvMatToOpInput;
  op::CvMatToOpOutput cvMatToOpOutput;
  op::PoseExtractorCaffe poseExtractorCaffe{poseModel, FLAGS_model_folder, FLAGS_num_gpu_start, {}, op::ScaleMode::ZeroToOne, enableGoogleLogging};
  op::PoseCpuRenderer poseRenderer{poseModel, (float)FLAGS_render_threshold, !FLAGS_disable_blending, (float)FLAGS_alpha_pose};
  op::OpOutputToCvMat opOutputToCvMat;
  op::FrameDisplayer frameDisplayer{"OpenPose tutorial", outputSize};

  /// INITIALIZE RESOURCES ON DESIRED THREAD
  poseExtractorCaffe.initializationOnThread();
  poseRenderer.initializationOnThread();

  // ==================================================================================================

  ////////////// POSE ESTIMATION AND RENDERING //////////////
  /// READ AND LOAD IMAGE
  cv::Mat inputImage = frame; // op::loadImage(FLAGS_image_path, CV_LOAD_IMAGE_COLOR);

  const op::Point<int> imageSize{inputImage.cols, inputImage.rows};

  /// GET DESIRED SCALE VALUES
  std::vector<double> scaleInputToNetInputs;
  std::vector<op::Point<int>> netInputSizes;
  double scaleInputToOutput;
  op::Point<int> outputResolution;

  std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution) = scaleAndSizeExtractor.extract(imageSize);

  /// FORMAT INPUT IMAGE TO OPENPOSE I/O FORMATS
  const auto netInputArray = cvMatToOpInput.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
  auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);

  /// ESTIMATE POSEKEYPOINTS
  poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
  const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
  *pose_keypoints = poseKeypoints;
  
  /// RENDER KEYPOINTS
  poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);

  /// OPENPOSE OUTPUT TO CV::MAT
  auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

  ////////////// SHOWING RESULTS AND CLOSING //////////////
  // frameDisplayer.displayFrame(outputImage, 0); // Alternative: cv::imshow(outputImage) + cv::waitKey(0)
  cv::imshow("pose_preview_", outputImage);
  cv::waitKey(1);
};

float endian_extract_value(cv::Mat frame, int row, int col) {
  unsigned short next_idx;
  if (col == frame.cols && row != frame.rows) next_idx = frame.at<unsigned short>(row+1, 0);
  else next_idx = frame.at<unsigned short>(row, col+1);
  return frame.at<unsigned short>(row, col) + (next_idx << 8);
};

float average_of_window(cv::Mat frame, int r, int row, int col) {
  float total = 0;
  int values = 0;
  for (int x = col-r; x < col+r; x+=2) {
    float value = 0;
    for(int y = row-r; y < row+r; y++) {
      if (x < 0 || x > frame.cols) value = 0;
      else if (y < 0 || y > frame.rows) value = 0;
      else {
	value = endian_extract_value(frame, y, x);
	values++;
      }
      
      total += value;
    }
  }

  return total/(float)values;

};

float estimateZ(int row, int col, sensor_msgs::Image depth_image)
{
  // TODO: grab the gaussian estimated value
  // TODO: do Z-shift to place skeleton inside person.
  // should use a LUT for the different body parts..

  // return average_of_window(frame, 5, row, col);
  return depth_image.data[row*depth_image.width + col*2] + depth_image.data[row*depth_image.width + col*2 +1]<<8;
  //  return (float)(frame.at<unsigned short>(row, col)+frame.at<unsigned short>(row, col+1)<<8);
};

void PoseEstimator::estimate_poses(cv::Mat frame, sensor_msgs::Image depth_frame) {  
  run_openpose(frame, &pose_keypoints_0);
  int _id = 0;
    

  for (auto person = 0; person < pose_keypoints_0.getSize(0); person++) {
    geometry_msgs::Point _coordinates[18];
      
    for (auto body_part = 0; body_part < pose_keypoints_0.getSize(1); body_part++) {

      /// Calculate 3D point:
      float u = (float)pose_keypoints_0[{person, body_part, 0}];
      float v = (float)pose_keypoints_0[{person, body_part, 1}];
      float score = (float)pose_keypoints_0[{person, body_part, 2}];

      float X = .0, Y = .0, Z = .0;
      // TODO: read from /base_name/camera_info
      float Cu = 479.75, Cv = 269.75, f = 540.68603515625;
      // the value of frame is of type 16UC1..
      //if (u > 0.0 && v > 0.0) {
      Z = estimateZ(u,v,depth_frame);
      X = (u - Cu) * Z / f;
      Y = (v - Cv) * Z / f;
      //}

      _coordinates[body_part] = assign_point(X, Y, Z);
    }

    people_0.push_back(Person(++_id, "camera_0_link", _coordinates));
  }

  // std::for_each(people_0.begin(), people_0.end(), *Person::draw_skeleton);
  for (std::vector<Person>::size_type i = 0; i < people_0.size(); i++)
    people_0[i].draw_skeleton();
    
};
