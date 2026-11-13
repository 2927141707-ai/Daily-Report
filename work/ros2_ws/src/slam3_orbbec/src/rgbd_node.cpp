#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <nlohmann/json.hpp>
#include "frame_writer.hpp"
#include "System.h"
#include "MapPoint.h"
#include "Map.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>

using Json = nlohmann::json;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class RGBDNode : public rclcpp::Node {
public:
  RGBDNode() : Node("orb_slam3_rgbd") {
    vocabulary_ = declare_parameter<std::string>("vocabulary", "");
    output_ = declare_parameter<std::string>("output_dir", "slam3_run");
    viewer_ = declare_parameter<bool>("viewer", true);
    record_dense_ = declare_parameter<bool>("record_dense", false);
    const int writer_capacity=declare_parameter<int>("writer_queue_size",8);
    snapshot_period_=declare_parameter<double>("snapshot_period",2.0);
    frame_prefix_=declare_parameter<std::string>("map_frame_prefix","slam_map_");
    camera_frame_=declare_parameter<std::string>("slam_camera_frame","slam_camera_optical");
    verify_camera_=declare_parameter<bool>("verify_camera_params",true);
    aligned_confirmed_=declare_parameter<bool>("aligned_depth_confirmed",false);
    const auto camera_node=declare_parameter<std::string>("camera_node","/camera/camera");
    if(writer_capacity<1 || snapshot_period_<0.2) throw std::runtime_error("Invalid queue or snapshot period");
    target_width_ = declare_parameter<int>("target_width", 640);
    fps_ = declare_parameter<int>("fps", 30);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    max_dt_ = declare_parameter<double>("max_sync_seconds", 0.02);
    max_depth_ = declare_parameter<double>("max_depth_m", 8.0);
    auto rgb_topic = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    auto info_topic = declare_parameter<std::string>("info_topic", "/camera/color/camera_info");
    if (!std::filesystem::is_regular_file(vocabulary_)) throw std::runtime_error("Vocabulary file not found");
    if(target_width_ < 160 || fps_ < 1 || depth_scale_ <= 0 || max_dt_ <= 0 || max_depth_ <= 0)
      throw std::runtime_error("Invalid image/depth/synchronization parameters");
    std::filesystem::create_directories(output_);
    if(record_dense_) {
      if(std::filesystem::exists(output_+"/frames.txt") || std::filesystem::exists(output_+"/intrinsics.json"))
        throw std::runtime_error("Dense recording requires a fresh output directory");
      if(!std::filesystem::create_directory(output_+"/.dense_session"))
        throw std::runtime_error("Output directory already claimed by another dense session");
      writer_=std::make_unique<FrameWriter>(output_,writer_capacity);
    }
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    snapshot_pub_=create_publisher<std_msgs::msg::String>("/orb_slam3/snapshot",rclcpp::QoS(1).reliable().transient_local());
    snapshot_service_=create_service<std_srvs::srv::Trigger>("/orb_slam3/get_snapshot",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response){
        try {
          if(!slam_||!writer_) throw std::runtime_error("SLAM/dense recording not initialized");
          writer_->drain();
          auto snap=makeSnapshot(false);
          response->message=snap.dump();response->success=true;
        }catch(const std::exception& e){response->success=false;response->message=e.what();}
      });
    snapshot_timer_=create_wall_timer(std::chrono::duration<double>(snapshot_period_),[this]{
      if(!slam_||!writer_) return;
      try {
        auto snapshot=makeSnapshot(false);
        writeJson(output_+"/latest_snapshot.json",snapshot);
        std_msgs::msg::String msg;msg.data=snapshot.dump();snapshot_pub_->publish(msg);
      }catch(const std::exception& e){RCLCPP_WARN(get_logger(),"Snapshot: %s",e.what());}
    });
    param_client_=std::make_shared<rclcpp::AsyncParametersClient>(this,camera_node);
    camera_timer_=create_wall_timer(std::chrono::seconds(2),[this]{checkCamera();});
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/orb_slam3/path", 1);
    state_pub_ = create_publisher<std_msgs::msg::String>("/orb_slam3/state", 10);
    info_sub_ = create_subscription<Info>(info_topic, rclcpp::SensorDataQoS(),
      [this](Info::ConstSharedPtr m) {
        if(slam_ && (m->width!=info_->width || m->height!=info_->height || m->k!=info_->k || m->d!=info_->d || m->distortion_model!=info_->distortion_model))
          calibration_changed_=true;
        if(!slam_) info_=m;
      });
    rgb_sub_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(5), rgb_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_dt_));
    sync_->registerCallback(std::bind(&RGBDNode::onFrames,this,std::placeholders::_1,std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      if(frames_ == 0) RCLCPP_WARN(get_logger(), "Waiting for synchronized RGB/depth and CameraInfo. Check camera, topics and alignment.");
      else RCLCPP_INFO(get_logger(), "Frames=%zu tracked=%zu state=%d map=%ld written=%zu dropped=%zu failed=%zu",frames_,tracked_,state_,map_id_,writer_?writer_->written.load():0,writer_?writer_->dropped.load():0,writer_?writer_->failed.load():0);
    });
    RCLCPP_INFO(get_logger(), "Waiting for %s + %s; outputs: %s",rgb_topic.c_str(),depth_topic.c_str(),output_.c_str());
  }
  void finish() {
    if(slam_) {
      slam_->Shutdown();
      if(record_dense_) {
        writer_->close();
        slam_->SaveKeyFrameTrajectoryTUM(output_+"/KeyFrameTrajectory_final.txt");
        writeJson(output_+"/final_snapshot.json",makeSnapshot(true));
      }
      slam_.reset();
    }
    if(trajectory_.is_open()) trajectory_.close();

  }
private:
  static void writeJson(const std::string& path,const Json& data) {
    std::ofstream f(path+".tmp");f<<data.dump(2)<<'\n';f.close();
    if(!f) throw std::runtime_error("Cannot write "+path);
    std::filesystem::rename(path+".tmp",path);
  }
  static std::string frameKey(const ORB_SLAM3::System::DenseKeyFrame& k) {
    return std::to_string(k.id)+"_"+std::to_string(std::llround(k.timestamp*1e9));
  }
  Json makeSnapshot(bool final) {
    auto snap=slam_->GetDenseSnapshot();
    Json result={{"schema",2},{"version",++snapshot_version_},{"final",final},
      {"run_dir",std::filesystem::absolute(output_).string()},{"active_map",snap.active_map},
      {"frame_prefix",frame_prefix_},{"intrinsics",intrinsics_},{"maps",Json::array()},
      {"writer",{{"written",writer_->written.load()},{"dropped",writer_->dropped.load()},
      {"failed",writer_->failed.load()},{"error",writer_->error()}}}};
    for(const auto& map:snap.maps) {
      Json m={{"id",map.id},{"change_index",map.change_index},{"keyframes",Json::array()}};
      for(const auto& k:map.keyframes) {
        const auto p=k.Twc.translation();const auto q=k.Twc.unit_quaternion();
        if(!k.Twc.matrix().allFinite()) continue;
        const auto key=frameKey(k);
        m["keyframes"].push_back({{"id",k.id},{"frame_id",k.frame_id},{"key",key},
          {"timestamp",k.timestamp},{"rgb","rgb/"+key+".png"},{"depth","depth/"+key+".png"},
          {"pose",{p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()}}});
      }
      result["maps"].push_back(m);
    }
    return result;
  }
  void checkCamera() {
    if(!verify_camera_) {camera_valid_=aligned_confirmed_;return;}
    if(param_pending_) return;
    if(!param_client_->service_is_ready()) {camera_valid_=false;return;}
    param_pending_=true;
    param_client_->get_parameters({"depth_registration","enable_depth_scale","enable_color_undistortion","enable_depth_undistortion"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
        param_pending_=false;
        try {
          auto p=future.get();
          camera_valid_=p.size()==4 && p[0].as_bool() && p[1].as_bool() && !p[2].as_bool() && !p[3].as_bool();
          if(!camera_valid_) RCLCPP_ERROR(get_logger(),"Driver must enable depth_registration/depth_scale and disable extra color/depth undistortion");
        }catch(const std::exception& e){camera_valid_=false;RCLCPP_ERROR(get_logger(),"Cannot verify camera parameters: %s",e.what());}
      });
  }
  void initialize(const Image& rgb) {
    if(!info_ || info_->k[0] <= 0 || info_->k[4] <= 0) throw std::runtime_error("CameraInfo missing or uncalibrated");
    if(info_->width != rgb.width || info_->height != rgb.height)
      throw std::runtime_error("CameraInfo/image dimensions differ; no guessed calibration is permitted");
    if(info_->binning_x > 1 || info_->binning_y > 1 || info_->roi.x_offset || info_->roi.y_offset)
      throw std::runtime_error("Cropped/binned CameraInfo is not supported by this bridge");
    raw_size_ = cv::Size(rgb.width,rgb.height);
    const double scale = std::min(1.0,static_cast<double>(target_width_)/rgb.width);
    image_size_ = cv::Size(std::lround(rgb.width*scale),std::lround(rgb.height*scale));
    cv::Mat K(3,3,CV_64F,const_cast<double*>(info_->k.data()));
    cv::Mat newK = K.clone();
    const double sx = double(image_size_.width)/raw_size_.width, sy = double(image_size_.height)/raw_size_.height;
    newK.at<double>(0,0)*=sx; newK.at<double>(0,2)*=sx;
    newK.at<double>(1,1)*=sy; newK.at<double>(1,2)*=sy;
    cv::Mat D(info_->d,true);
    if(info_->distortion_model == "equidistant") {
      if(info_->d.size()!=4) throw std::runtime_error("Invalid equidistant distortion coefficients");
      cv::fisheye::initUndistortRectifyMap(K,D,cv::Mat::eye(3,3,CV_64F),newK,image_size_,CV_32FC1,mapx_,mapy_);
    } else if(info_->distortion_model.empty() || info_->distortion_model=="plumb_bob" || info_->distortion_model=="rational_polynomial") {
      cv::initUndistortRectifyMap(K,D,cv::Mat(),newK,image_size_,CV_32FC1,mapx_,mapy_);
    } else throw std::runtime_error("Unsupported camera distortion model");
    const auto settings = output_ + "/camera.yaml";
    std::ofstream f(settings);
    if(!f) throw std::runtime_error("Cannot write camera.yaml");
    f << std::fixed << std::setprecision(9);
    f << "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"PinHole\"\n";
    f << "Camera1.fx: " << newK.at<double>(0,0) << "\nCamera1.fy: " << newK.at<double>(1,1)
      << "\nCamera1.cx: " << newK.at<double>(0,2) << "\nCamera1.cy: " << newK.at<double>(1,2) << '\n';
    f << "Camera1.k1: 0.0\nCamera1.k2: 0.0\nCamera1.p1: 0.0\nCamera1.p2: 0.0\nCamera1.k3: 0.0\n";
    f << "Camera.width: " << image_size_.width << "\nCamera.height: " << image_size_.height << "\nCamera.fps: " << fps_ << '\n';
    f << "Camera.RGB: 0\nRGBD.DepthMapFactor: 1.0\n";
    // RGB-D virtual baseline: 0.05 * 40 = 2 m close-point threshold; not a measured camera extrinsic.
    f << "Stereo.b: 0.05\nStereo.ThDepth: 40.0\n";
    f << "ORBextractor.nFeatures: 1500\nORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\nORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n";
    f << "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\nViewer.GraphLineWidth: 0.9\nViewer.PointSize: 2.0\nViewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\nViewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -1.8\nViewer.ViewpointF: 500.0\n";
    f.close();
    intrinsics_={{"width",image_size_.width},{"height",image_size_.height},
      {"fx",newK.at<double>(0,0)},{"fy",newK.at<double>(1,1)},
      {"cx",newK.at<double>(0,2)},{"cy",newK.at<double>(1,2)},
      {"depth_units_per_metre",1000},{"rectified",true}};
    writeJson(output_+"/intrinsics.json",intrinsics_);
    Json config;
    for(const auto& name:list_parameters({},10).names) {
      config[name]=get_parameter(name).value_to_string();
    }
    writeJson(output_+"/runtime_config.json",config);
    // Keep source calibration for audit. Distortion is removed from both aligned inputs with the same map.
    std::ofstream audit(output_+"/camera_info.txt");
    audit << "frame_id=" << info_->header.frame_id << "\nwidth=" << info_->width << " height=" << info_->height << "\nmodel=" << info_->distortion_model << "\nK=";
    for(auto v:info_->k) audit << std::setprecision(17) << v << ' ';
    audit << "\nD="; for(auto v:info_->d) audit << std::setprecision(17) << v << ' ';
    audit << "\n16UC1_depth_scale=" << depth_scale_ << "\n32FC1_depth_unit=metres\n";
    slam_ = std::make_unique<ORB_SLAM3::System>(vocabulary_,settings,ORB_SLAM3::System::RGBD,viewer_);
    RCLCPP_INFO(get_logger(),"SLAM initialized at %dx%d using driver calibration",image_size_.width,image_size_.height);
  }
  void onFrames(const Image::ConstSharedPtr& rgb,const Image::ConstSharedPtr& depth) {
    if(!camera_valid_ || calibration_changed_) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Input rejected: verify driver alignment/depth-scale parameters or changed calibration");return;
    }
    if(!info_) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for color CameraInfo"); return; }
    const double t=rclcpp::Time(rgb->header.stamp).seconds(), td=rclcpp::Time(depth->header.stamp).seconds();
    if(t<=last_stamp_ || std::abs(t-td)>max_dt_) return;
    if(rgb->width!=depth->width || rgb->height!=depth->height) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"RGB/depth dimensions differ: enable depth_registration"); return;
    }
    try {
      if(!slam_) { initialize(*rgb); return; } // Drop the stale image held during vocabulary loading.
      if(cv::Size(rgb->width,rgb->height)!=raw_size_) throw std::runtime_error("Stream resolution changed; restart SLAM");
      auto color=cv_bridge::toCvShare(rgb,sensor_msgs::image_encodings::BGR8);
      auto depth_cv=cv_bridge::toCvShare(depth);
      cv::Mat metres;
      if(depth->encoding==sensor_msgs::image_encodings::TYPE_16UC1) depth_cv->image.convertTo(metres,CV_32F,depth_scale_);
      else if(depth->encoding==sensor_msgs::image_encodings::TYPE_32FC1) metres=depth_cv->image.clone();
      else throw std::runtime_error("Only 16UC1 or 32FC1 depth is supported");
      for(int y=0;y<metres.rows;++y) for(int x=0;x<metres.cols;++x) {
        float& z=metres.at<float>(y,x); if(!std::isfinite(z)||z<=0||z>max_depth_) z=0;
      }
      cv::Mat rgb_rect,depth_rect;
      cv::remap(color->image,rgb_rect,mapx_,mapy_,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
      cv::remap(metres,depth_rect,mapx_,mapy_,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
      if(cv::countNonZero(depth_rect) < 100) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Too few valid depth pixels"); return; }
      last_stamp_=t;
      auto Tcw=slam_->TrackRGBD(rgb_rect,depth_rect,t);
      if(record_dense_) {
        ORB_SLAM3::System::DenseKeyFrame keyframe;
        if(slam_->GetCurrentDenseKeyFrame(keyframe)) {
          cv::Mat depth_mm;depth_rect.convertTo(depth_mm,CV_16U,1000.0);
          if(!writer_->submit({frameKey(keyframe),t,rgb_rect.clone(),depth_mm}))
            RCLCPP_WARN(get_logger(),"RGB-D queue full: dropping this keyframe image; missing coverage will be reported");
        }
      }
      ++frames_; state_=slam_->GetTrackingState();
      std_msgs::msg::String state_msg;
      state_msg.data=state_==ORB_SLAM3::Tracking::OK ? "TRACKING_OK" : "NOT_TRACKING state="+std::to_string(state_);
      state_pub_->publish(state_msg);
      if(state_!=ORB_SLAM3::Tracking::OK) return;
      long id=-1;
      for(auto* p:slam_->GetTrackedMapPoints()) if(p && !p->isBad() && p->GetMap()) { id=p->GetMap()->GetId(); break; }
      if(id<0) return;
      if(id!=map_id_) {
        map_id_=id; path_.poses.clear();
        if(trajectory_.is_open()) trajectory_.close();
        trajectory_.open(output_+"/trajectory_live_map_"+std::to_string(id)+".txt",std::ios::app);
        if(!trajectory_) throw std::runtime_error("Cannot write trajectory");
        RCLCPP_INFO(get_logger(),"Active map %ld; path reset",id);
      }
      auto Twc=Tcw.inverse();
      if(!Twc.matrix().allFinite()) return;
      const auto pos=Twc.translation(); const auto q=Twc.unit_quaternion();
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp=rgb->header.stamp; pose.header.frame_id=frame_prefix_+std::to_string(id);
      pose.pose.position.x=pos.x(); pose.pose.position.y=pos.y(); pose.pose.position.z=pos.z();
      pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
      pose_pub_->publish(pose); ++tracked_;
      geometry_msgs::msg::TransformStamped transform;
      transform.header=pose.header;transform.child_frame_id=camera_frame_;
      transform.transform.translation.x=pos.x();transform.transform.translation.y=pos.y();transform.transform.translation.z=pos.z();
      transform.transform.rotation=pose.pose.orientation;tf_->sendTransform(transform);
      if(tracked_%3==0) {
        path_.header=pose.header; path_.poses.push_back(pose);
        if(path_.poses.size()>10000) path_.poses.erase(path_.poses.begin(),path_.poses.begin()+1000);
        path_pub_->publish(path_);
      }
      trajectory_ << std::fixed << std::setprecision(9) << t << ' ' << pos.x() << ' ' << pos.y() << ' ' << pos.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      if(tracked_%30==0) trajectory_.flush();
    } catch(const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"RGB-D processing error: %s",e.what());
    }
  }
  std::string vocabulary_,output_,frame_prefix_,camera_frame_;
  Json intrinsics_;double snapshot_period_;size_t snapshot_version_=0;
  bool verify_camera_=true,aligned_confirmed_=false,camera_valid_=false,param_pending_=false,calibration_changed_=false;
  std::unique_ptr<FrameWriter> writer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr snapshot_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr snapshot_pub_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_,camera_timer_;
  bool viewer_,record_dense_; int target_width_,fps_,state_=-1; double depth_scale_,max_dt_,max_depth_,last_stamp_=-1;
  size_t frames_=0,tracked_=0; long map_id_=-1;
  cv::Size raw_size_,image_size_; cv::Mat mapx_,mapy_;
  Info::ConstSharedPtr info_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::ofstream trajectory_; nav_msgs::msg::Path path_;
  message_filters::Subscriber<Image> rgb_sub_,depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<RGBDNode>();
    rclcpp::spin(node);
    node->finish();
    rclcpp::shutdown();
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
