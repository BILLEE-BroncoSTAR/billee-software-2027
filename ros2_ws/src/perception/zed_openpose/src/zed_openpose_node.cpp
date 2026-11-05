// ROS 2 port of stereolabs/zed-openpose (src/main.cpp): ZED 2 + CMU OpenPose on the GPU.
//
// Same pipeline as the sample: a ZED thread grabs the left image + XYZ point cloud and prepares
// OpenPose's input while an OpenPose thread runs the previous frame through
// op::PoseExtractorCaffe (CUDA); keypoints are lifted to 3D from the ZED's XYZ (skeleton_3d.hpp).
// The sample's OpenGL / OpenCV windows become topics, placed in the rover's TF tree:
//   /people/skeletons              people_msgs/Skeletons  everyone, 2D + 3D joints in target_frame
//   /people/markers                MarkerArray            3D skeletons (the sample's OpenGL view)
//   /people/debug_image(/compressed)                      OpenPose's rendering (opencv_display)
//   /zed/points                    PointCloud2            optional (ogl_ptcloud)
//   /zed/depth_image               Image                  optional (depth_display)
// The sample's floor-plane alignment is replaced by TF: the ZED reports points in its left camera
// frame with ROS axes, and the URDF says where camera_link sits on the rover.
//
// enable_pose:=false is the camera-only mode: no OpenPose thread, no model load, no /people/*;
// the ZED thread grabs at the camera's own rate and only publishes /zed/points (+ depth).
//
// Copyright (c) 2019 Stereolabs, MIT License (LICENSE.zed-openpose). ROS 2 port for BILLEE.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <openpose/headers.hpp>
#include <sl/Camera.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "people_msgs/msg/skeletons.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "zed_openpose/skeleton_3d.hpp"

namespace zed_openpose
{

namespace
{

sl::RESOLUTION parse_resolution(const std::string & name)
{
  static const std::map<std::string, sl::RESOLUTION> names = {
    {"HD2K", sl::RESOLUTION::HD2K}, {"HD1080", sl::RESOLUTION::HD1080},
    {"HD720", sl::RESOLUTION::HD720}, {"VGA", sl::RESOLUTION::VGA},
    {"AUTO", sl::RESOLUTION::AUTO}};
  const auto it = names.find(name);
  if (it == names.end()) {
    throw std::invalid_argument("camera_resolution must be HD2K, HD1080, HD720, VGA or AUTO");
  }
  return it->second;
}

sl::DEPTH_MODE parse_depth_mode(const std::string & name)
{
  static const std::map<std::string, sl::DEPTH_MODE> names = {
    {"PERFORMANCE", sl::DEPTH_MODE::PERFORMANCE}, {"QUALITY", sl::DEPTH_MODE::QUALITY},
    {"ULTRA", sl::DEPTH_MODE::ULTRA}, {"NEURAL_LIGHT", sl::DEPTH_MODE::NEURAL_LIGHT},
    {"NEURAL", sl::DEPTH_MODE::NEURAL}, {"NEURAL_PLUS", sl::DEPTH_MODE::NEURAL_PLUS}};
  const auto it = names.find(name);
  if (it == names.end()) {
    throw std::invalid_argument(
      "depth_mode must be PERFORMANCE, QUALITY, ULTRA, NEURAL_LIGHT, NEURAL or NEURAL_PLUS");
  }
  return it->second;
}

// cv::Mat view of a ZED CPU matrix (the sample's slMat2cvMat, for the types used here).
cv::Mat sl_to_cv(sl::Mat & input)
{
  const int type = input.getDataType() == sl::MAT_TYPE::U8_C4 ? CV_8UC4 : CV_32FC4;
  return cv::Mat(
    static_cast<int>(input.getHeight()), static_cast<int>(input.getWidth()), type,
    input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

geometry_msgs::msg::Point nan_point()
{
  const Xyz n = nan_xyz();
  geometry_msgs::msg::Point out;
  out.x = n[0];
  out.y = n[1];
  out.z = n[2];
  return out;
}

geometry_msgs::msg::Point to_point(const tf2::Transform & tf, const Xyz & p)
{
  geometry_msgs::msg::Point out;
  const tf2::Vector3 q = tf * tf2::Vector3(p[0], p[1], p[2]);
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  return out;
}

}  // namespace

// What the ZED thread hands to the OpenPose thread: one frame, its XYZ and OpenPose's inputs.
struct Frame
{
  rclcpp::Time stamp;
  cv::Mat bgr;                              // OpenPose's input image
  sl::Mat xyz;                              // XYZ (+ colour) for that same image, deep copy
  std::vector<op::Array<float>> net_input;  // cvMatToOpInput.createArray(...)
  op::Array<float> output;                  // cvMatToOpOutput.createArray(...), for rendering
};

class ZedOpenPoseNode : public rclcpp::Node
{
public:
  ZedOpenPoseNode()
  : Node("zed_openpose")
  {
    // The sample's gflags, same names and defaults (except the camera: see README).
    model_pose_ = declare_parameter<std::string>("model_pose", "BODY_25");
    model_folder_ = declare_parameter<std::string>("model_folder", "models/openpose/");
    net_resolution_ = declare_parameter<std::string>("net_resolution", "320x240");
    output_resolution_flag_ = declare_parameter<std::string>("output_resolution", "-1x-1");
    num_gpu_start_ = declare_parameter<int>("num_gpu_start", 0);
    scale_gap_ = declare_parameter<double>("scale_gap", 0.3);
    scale_number_ = declare_parameter<int>("scale_number", 1);
    disable_blending_ = declare_parameter<bool>("disable_blending", false);
    render_threshold_ = declare_parameter<double>("render_threshold", 0.12);
    alpha_pose_ = declare_parameter<double>("alpha_pose", 0.6);
    svo_path_ = declare_parameter<std::string>("svo_path", "");
    publish_cloud_ = declare_parameter<bool>("publish_cloud", false);    // ogl_ptcloud
    publish_depth_ = declare_parameter<bool>("publish_depth", false);    // depth_display
    enable_pose_ = declare_parameter<bool>("enable_pose", true);          // false: camera only
    cloud_max_hz_ = declare_parameter<double>("cloud_max_hz", 10.0);     // 0 = every frame
    // ROS additions.
    camera_resolution_ = declare_parameter<std::string>("camera_resolution", "HD720");
    depth_mode_ = declare_parameter<std::string>("depth_mode", "ULTRA");
    image_width_ = declare_parameter<int>("image_width", 720);
    image_height_ = declare_parameter<int>("image_height", 405);
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
    zed_frame_ = declare_parameter<std::string>("zed_frame", "zed_left_camera_frame");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    cloud_step_ = declare_parameter<int>("cloud_step", 4);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);

    if (!model_folder_.empty() && model_folder_.back() != '/') {
      model_folder_ += '/';  // OpenPose appends "pose/<model>/..." to it as is
    }
    if (alpha_pose_ < 0. || alpha_pose_ > 1.) {
      throw std::invalid_argument("alpha_pose must be in [0, 1]");
    }
    if (scale_gap_ <= 0. && scale_number_ > 1) {
      throw std::invalid_argument("scale_gap must be > 0 when scale_number > 1");
    }
    if (!enable_pose_ && !publish_cloud_ && !publish_depth_) {
      throw std::invalid_argument(
        "enable_pose, publish_cloud and publish_depth are all false: nothing to publish");
    }

    open_zed();

    if (enable_pose_) {
      skeletons_pub_ = create_publisher<people_msgs::msg::Skeletons>("/people/skeletons", 10);
      markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/people/markers", 1);
      image_pub_ = create_publisher<sensor_msgs::msg::Image>("/people/debug_image", 1);
      compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/people/debug_image/compressed", 1);
    }
    if (publish_cloud_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/zed/points", 1);
    }
    if (publish_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/zed/depth_image", 1);
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // OpenPose configuration, as in the sample's main().
    output_size_ = op::flagsToPoint(op::String(output_resolution_flag_), "-1x-1");
    net_input_size_ = op::flagsToPoint(op::String(net_resolution_), "-1x368");
    pose_model_ = op::flagsToPoseModel(op::String(model_pose_));
    image_size_ = op::Point<int>{image_width_, image_height_};

    zed_thread_ = std::thread(&ZedOpenPoseNode::run, this);
    if (enable_pose_) {
      openpose_thread_ = std::thread(&ZedOpenPoseNode::findpose, this);
    } else {
      RCLCPP_INFO(get_logger(), "Camera-only mode (enable_pose:=false): OpenPose not loaded");
    }
  }

  ~ZedOpenPoseNode() override
  {
    quit_ = true;
    if (openpose_thread_.joinable()) {openpose_thread_.join();}
    if (zed_thread_.joinable()) {zed_thread_.join();}
    zed_.close();
  }

private:
  void open_zed()
  {
    sl::InitParameters init;
    init.camera_resolution = parse_resolution(camera_resolution_);
    init.depth_mode = parse_depth_mode(depth_mode_);
    init.coordinate_units = sl::UNIT::METER;
    // ROS axes (x forward, y left, z up) instead of the sample's RIGHT_HANDED_Y_UP.
    init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init.sdk_verbose = 0;
    init.svo_real_time_mode = true;
    if (!svo_path_.empty()) {
      RCLCPP_INFO(get_logger(), "Opening %s", svo_path_.c_str());
      init.input.setFromSVOFile(svo_path_.c_str());
    }
    const sl::ERROR_CODE err = zed_.open(init);
    if (err != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(
        std::string("ZED open failed: ") + sl::toString(err).c_str() +
        (svo_path_.empty() ? ". Is the ZED plugged into the Jetson (lsusb | grep 2b03)?" : ""));
    }
    const auto info = zed_.getCameraInformation();
    RCLCPP_INFO(
      get_logger(), "ZED %s #%u open: %s, depth %s, OpenPose input %dx%d",
      sl::toString(info.camera_model).c_str(), info.serial_number, camera_resolution_.c_str(),
      depth_mode_.c_str(), image_width_, image_height_);

    // Points come in the left camera's frame: half a baseline to the left (+y) of camera_link.
    geometry_msgs::msg::TransformStamped lens;
    lens.header.stamp = now();
    lens.header.frame_id = camera_frame_;
    lens.child_frame_id = zed_frame_;
    lens.transform.translation.y =
      info.camera_configuration.calibration_parameters.getCameraBaseline() / 2.0;
    lens.transform.rotation.w = 1.0;
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    static_tf_->sendTransform(lens);
  }

  // The sample's run(): grab while OpenPose works on the previous frame.
  void run()
  {
    sl::RuntimeParameters rt;
    rt.enable_depth = true;
    rt.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;

    sl::Mat img_buffer, xyz_buffer, depth_img_buffer;
    cv::Mat input_bgr;
    op::ScaleAndSizeExtractor scale_and_size{
      net_input_size_, 1.f, output_size_, scale_number_, scale_gap_};
    op::CvMatToOpInput cv_to_input{pose_model_};
    op::CvMatToOpOutput cv_to_output;

    const sl::Resolution resolution(image_width_, image_height_);
    std::vector<op::Point<int>> net_input_sizes;
    std::tie(scale_input_to_net_inputs_, net_input_sizes, scale_input_to_output_, output_resolution_) =
      scale_and_size.extract(image_size_);
    const double cloud_period = cloud_max_hz_ > 0. ? 1.0 / cloud_max_hz_ : 0.;
    bool cloud_sent = false;
    rclcpp::Time last_cloud;

    while (!quit_ && rclcpp::ok()) {
      // With OpenPose, grab only once it has taken the last frame; camera-only grabs freely.
      if (enable_pose_ && !need_new_image_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const sl::ERROR_CODE grabbed = zed_.grab(rt);
      if (grabbed == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
        RCLCPP_INFO(get_logger(), "End of %s", svo_path_.c_str());
        quit_ = true;
        rclcpp::shutdown();
        break;
      }
      if (grabbed != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "ZED grab: %s", sl::toString(grabbed).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      // Live: the image's capture time; SVO: now, so TF lookups don't fall behind the recording.
      const rclcpp::Time stamp = svo_path_.empty() ?
        rclcpp::Time(
        static_cast<int64_t>(zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds()),
        RCL_ROS_TIME) : now();
      // XYZBGRA: the sample's XYZRGBA with the colour bytes in ROS's PointCloud2 rgb order.
      zed_.retrieveMeasure(xyz_buffer, sl::MEASURE::XYZBGRA, sl::MEM::CPU, resolution);

      if (publish_depth_) {
        zed_.retrieveImage(depth_img_buffer, sl::VIEW::DEPTH, sl::MEM::CPU, resolution);
        publish_image(*depth_pub_, sl_to_cv(depth_img_buffer), "bgra8", stamp, zed_frame_);
      }
      if (publish_cloud_ && (!cloud_sent || (stamp - last_cloud).seconds() >= cloud_period)) {
        publish_cloud(xyz_buffer, stamp);
        last_cloud = stamp;
        cloud_sent = true;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "ZED %.1f fps", zed_.getCurrentFPS());
      if (!enable_pose_) {continue;}

      auto frame = std::make_shared<Frame>();
      frame->stamp = stamp;
      zed_.retrieveImage(img_buffer, sl::VIEW::LEFT, sl::MEM::CPU, resolution);
      xyz_buffer.copyTo(frame->xyz, sl::COPY_TYPE::CPU_CPU);
      cv::cvtColor(sl_to_cv(img_buffer), input_bgr, cv::COLOR_BGRA2BGR);
      frame->bgr = input_bgr.clone();
      frame->output = cv_to_output.createArray(
        OP_CV2OPCONSTMAT(frame->bgr), scale_input_to_output_, output_resolution_);
      frame->net_input = cv_to_input.createArray(
        OP_CV2OPCONSTMAT(frame->bgr), scale_input_to_net_inputs_, net_input_sizes);

      {
        std::lock_guard<std::mutex> lock(data_in_mtx_);
        pending_ = frame;
        need_new_image_ = false;
      }
    }
    quit_ = true;
  }

  // The sample's findpose(): CUDA OpenPose on the newest frame, then lift and publish it.
  void findpose()
  {
    try {
      op::PoseExtractorCaffe extractor{pose_model_, model_folder_, num_gpu_start_};
      extractor.initializationOnThread();
      op::PoseCpuRenderer renderer{
        pose_model_, static_cast<float>(render_threshold_), !disable_blending_,
        static_cast<float>(alpha_pose_)};
      renderer.initializationOnThread();
      op::OpOutputToCvMat output_to_cv;
      RCLCPP_INFO(get_logger(), "OpenPose %s loaded from %s", model_pose_.c_str(),
        model_folder_.c_str());

      while (!quit_ && rclcpp::ok()) {
        std::shared_ptr<Frame> frame;
        {
          std::lock_guard<std::mutex> lock(data_in_mtx_);
          frame.swap(pending_);
          if (frame) {need_new_image_ = true;}  // let the ZED thread grab the next one now
        }
        if (!frame) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        extractor.forwardPass(frame->net_input, image_size_, scale_input_to_net_inputs_);
        const op::Array<float> keypoints = extractor.getPoseKeypoints();
        publish_people(*frame, keypoints);

        if (!frame->output.empty()) {
          renderer.renderPose(frame->output, keypoints, static_cast<float>(scale_input_to_output_));
          const op::Matrix rendered = output_to_cv.formatToCvMat(frame->output);
          publish_debug_image(OP_OP2CVCONSTMAT(rendered), frame->stamp);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "OpenPose: %s", e.what());
      quit_ = true;
      rclcpp::shutdown();
    }
  }

  void publish_people(Frame & frame, const op::Array<float> & keypoints)
  {
    people_msgs::msg::Skeletons msg;
    msg.header.stamp = frame.stamp;
    msg.header.frame_id = target_frame_;
    msg.model = model_pose_;
    msg.image_width = static_cast<uint32_t>(image_width_);
    msg.image_height = static_cast<uint32_t>(image_height_);

    tf2::Transform to_target;
    bool have_tf = true;
    try {
      const auto t = tf_buffer_->lookupTransform(
        target_frame_, zed_frame_, frame.stamp, rclcpp::Duration::from_seconds(0.1));
      const auto & r = t.transform.rotation;
      const auto & p = t.transform.translation;
      to_target = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(p.x, p.y, p.z));
    } catch (const tf2::TransformException & e) {
      have_tf = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No TF %s <- %s: %s",
        target_frame_.c_str(), zed_frame_.c_str(), e.what());
    }

    const auto lookup = [&frame](int i, int j) {
        sl::float4 v;
        if (frame.xyz.getValue<sl::float4>(i, j, &v, sl::MEM::CPU) != sl::ERROR_CODE::SUCCESS) {
          return nan_xyz();
        }
        return Xyz{v.x, v.y, v.z};
      };

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = msg.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const int people = keypoints.empty() ? 0 : keypoints.getSize(0);
    const int parts = keypoints.empty() ? 0 : keypoints.getSize(1);
    for (int person = 0; person < people; ++person) {
      std::vector<std::array<float, 3>> row(parts);
      people_msgs::msg::Skeleton skeleton;
      for (int k = 0; k < parts; ++k) {
        row[k] = {keypoints[{person, k, 0}], keypoints[{person, k, 1}], keypoints[{person, k, 2}]};
        skeleton.u.push_back(row[k][0]);
        skeleton.v.push_back(row[k][1]);
        skeleton.confidence.push_back(row[k][2]);
      }
      const Person3D lifted = lift_person(row, static_cast<float>(render_threshold_), lookup);
      for (const auto & joint : lifted.joints) {
        skeleton.joints.push_back(have_tf ? to_point(to_target, joint) : nan_point());
      }
      skeleton.centroid = have_tf ? to_point(to_target, lifted.centroid) : nan_point();
      msg.people.push_back(skeleton);

      if (have_tf) {
        visualization_msgs::msg::Marker limbs;
        limbs.header = msg.header;
        limbs.ns = "skeleton";
        limbs.id = person;
        limbs.type = visualization_msgs::msg::Marker::LINE_LIST;
        limbs.pose.orientation.w = 1.0;
        limbs.scale.x = 0.03;
        limbs.color.r = 1.0;   // the sample's generateColor(): red
        limbs.color.a = 1.0;
        for (const auto & [a, b] : lifted.limbs) {
          limbs.points.push_back(skeleton.joints[a]);
          limbs.points.push_back(skeleton.joints[b]);
        }
        markers.markers.push_back(limbs);
      }
    }
    skeletons_pub_->publish(msg);
    markers_pub_->publish(markers);
  }

  void publish_debug_image(const cv::Mat & bgr, const rclcpp::Time & stamp)
  {
    publish_image(*image_pub_, bgr, "bgr8", stamp, zed_frame_);
    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header.stamp = stamp;
    jpeg.header.frame_id = zed_frame_;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", bgr, jpeg.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    compressed_pub_->publish(jpeg);
  }

  void publish_image(
    rclcpp::Publisher<sensor_msgs::msg::Image> & pub, const cv::Mat & image,
    const std::string & encoding, const rclcpp::Time & stamp, const std::string & frame_id)
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = encoding;
    msg.step = static_cast<uint32_t>(image.cols * image.elemSize());
    msg.data.resize(msg.step * msg.height);
    for (int row = 0; row < image.rows; ++row) {
      std::memcpy(&msg.data[row * msg.step], image.ptr(row), msg.step);
    }
    pub.publish(msg);
  }

  // The sample's fill_ptcloud, decimated by cloud_step, in the ZED's frame.
  void publish_cloud(sl::Mat & xyz, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = zed_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    const int step = std::max(1, cloud_step_);
    const int width = static_cast<int>(xyz.getWidth());
    const int height = static_cast<int>(xyz.getHeight());
    modifier.resize(static_cast<size_t>((width / step + 1) * (height / step + 1)));
    sensor_msgs::PointCloud2Iterator<float> out(cloud, "x");
    size_t count = 0;
    for (int j = 0; j < height; j += step) {
      for (int i = 0; i < width; i += step) {
        sl::float4 v;
        xyz.getValue<sl::float4>(i, j, &v, sl::MEM::CPU);
        if (!std::isfinite(v.z)) {continue;}
        out[0] = v.x;
        out[1] = v.y;
        out[2] = v.z;
        out[3] = v.w;  // packed BGRA colour
        ++out;
        ++count;
      }
    }
    modifier.resize(count);
    cloud_pub_->publish(cloud);
  }

  // Parameters.
  std::string model_pose_, model_folder_, net_resolution_, output_resolution_flag_, svo_path_;
  std::string camera_resolution_, depth_mode_, camera_frame_, zed_frame_, target_frame_;
  int num_gpu_start_, scale_number_, image_width_, image_height_, cloud_step_, jpeg_quality_;
  double scale_gap_, render_threshold_, alpha_pose_, cloud_max_hz_;
  bool disable_blending_, publish_cloud_, publish_depth_, enable_pose_;

  // ZED + OpenPose state shared by the two threads (the sample's globals).
  sl::Camera zed_;
  op::PoseModel pose_model_;
  op::Point<int> image_size_, output_size_, net_input_size_, output_resolution_;
  std::vector<double> scale_input_to_net_inputs_;
  double scale_input_to_output_ = 1.0;
  std::mutex data_in_mtx_;
  std::shared_ptr<Frame> pending_;
  std::atomic<bool> need_new_image_{true};
  std::atomic<bool> quit_{false};
  std::thread zed_thread_, openpose_thread_;

  rclcpp::Publisher<people_msgs::msg::Skeletons>::SharedPtr skeletons_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

}  // namespace zed_openpose

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<zed_openpose::ZedOpenPoseNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_openpose"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
