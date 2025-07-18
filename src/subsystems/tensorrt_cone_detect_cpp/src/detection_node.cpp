#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "oak_cone_detect_interfaces/msg/cone2_d.hpp"
#include "oak_cone_detect_interfaces/msg/cone_array2_d.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

#include <algorithm>

// --- Simple SORT tracker implementation (ported from Python) ---
static float iou(const cv::Rect2f &a, const cv::Rect2f &b) {
  float xx1 = std::max(a.x, b.x);
  float yy1 = std::max(a.y, b.y);
  float xx2 = std::min(a.x + a.width, b.x + b.width);
  float yy2 = std::min(a.y + a.height, b.y + b.height);
  float w = std::max(0.f, xx2 - xx1);
  float h = std::max(0.f, yy2 - yy1);
  float inter = w * h;
  float uni = a.width * a.height + b.width * b.height - inter + 1e-6f;
  return inter / uni;
}

static cv::Mat convert_bbox_to_z(const cv::Rect2f &bbox) {
  float w = bbox.width;
  float h = bbox.height;
  float x = bbox.x + w / 2.0f;
  float y = bbox.y + h / 2.0f;
  float s = w * h;
  float r = w / (h + 1e-6f);
  cv::Mat z(4, 1, CV_32F);
  z.at<float>(0) = x;
  z.at<float>(1) = y;
  z.at<float>(2) = s;
  z.at<float>(3) = r;
  return z;
}

static cv::Rect2f convert_x_to_bbox(const cv::Mat &x) {
  float w = std::sqrt(x.at<float>(2) * x.at<float>(3));
  float h = x.at<float>(2) / (w + 1e-6f);
  float cx = x.at<float>(0);
  float cy = x.at<float>(1);
  return cv::Rect2f(cx - w / 2.f, cy - h / 2.f, w, h);
}

class KalmanBoxTracker {
public:
  KalmanBoxTracker(const cv::Rect2f &bbox) : kf_(7, 4, 0) {
    float dt = 1.f;
    kf_.transitionMatrix = (cv::Mat_<float>(7, 7) <<
      1,0,0,0,dt,0,0,
      0,1,0,0,0,dt,0,
      0,0,1,0,0,0,dt,
      0,0,0,1,0,0,0,
      0,0,0,0,1,0,0,
      0,0,0,0,0,1,0,
      0,0,0,0,0,0,1);
    kf_.measurementMatrix = (cv::Mat_<float>(4,7) <<
      1,0,0,0,0,0,0,
      0,1,0,0,0,0,0,
      0,0,1,0,0,0,0,
      0,0,0,1,0,0,0);
    setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));
    kf_.processNoiseCov.at<float>(6,6) *= 0.01f;
    for(int i=4;i<7;i++) kf_.processNoiseCov.at<float>(i,i) *= 0.01f;
    setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1));
    kf_.measurementNoiseCov.at<float>(2,2) *= 10.f;
    kf_.measurementNoiseCov.at<float>(3,3) *= 10.f;
    setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
    for(int i=4;i<7;i++) kf_.errorCovPost.at<float>(i,i) *= 1000.f;
    kf_.errorCovPost *= 10.f;
    cv::Mat z = convert_bbox_to_z(bbox);
    kf_.statePost.at<float>(0) = z.at<float>(0);
    kf_.statePost.at<float>(1) = z.at<float>(1);
    kf_.statePost.at<float>(2) = z.at<float>(2);
    kf_.statePost.at<float>(3) = z.at<float>(3);
    kf_.statePost.at<float>(4) = 0;
    kf_.statePost.at<float>(5) = 0;
    kf_.statePost.at<float>(6) = 0;
    time_since_update_ = 0;
    id_ = count_++;
    hits_ = 1;
    hit_streak_ = 1;
    age_ = 0;
  }

  void update(const cv::Rect2f &bbox) {
    cv::Mat z = convert_bbox_to_z(bbox);
    kf_.correct(z);
    time_since_update_ = 0;
    hits_++;
    hit_streak_++;
  }

  cv::Rect2f predict() {
    if ((kf_.statePost.at<float>(6) + kf_.statePost.at<float>(2)) <= 0)
      kf_.statePost.at<float>(6) = 0;
    cv::Mat p = kf_.predict();
    age_++;
    if (time_since_update_ > 0)
      hit_streak_ = 0;
    time_since_update_++;
    return convert_x_to_bbox(p);
  }

  cv::Rect2f get_state() const { return convert_x_to_bbox(kf_.statePost); }

  int id_{0};
  int time_since_update_{0};
  int hits_{0};
  int hit_streak_{0};
  int age_{0};

private:
  cv::KalmanFilter kf_;
  static int count_;
};

int KalmanBoxTracker::count_ = 0;

class SortTracker {
public:
  SortTracker(int max_age = 1, int min_hits = 3, float iou_threshold = 0.3f)
      : max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold),
        frame_count_(0) {}

  std::vector<std::array<float, 5>> update(const std::vector<cv::Rect2f> &dets) {
    frame_count_++;
    std::vector<cv::Rect2f> trks(trackers_.size());
    for (size_t i = 0; i < trackers_.size(); ++i)
      trks[i] = trackers_[i]->predict();

    std::vector<int> to_del;
    for (size_t i = 0; i < trks.size(); ++i) {
      if (std::isnan(trks[i].x))
        to_del.push_back(i);
    }
    for (int i = to_del.size() - 1; i >= 0; --i) {
      trackers_.erase(trackers_.begin() + to_del[i]);
      trks.erase(trks.begin() + to_del[i]);
    }

    size_t D = dets.size(), T = trks.size();
    std::vector<std::vector<float>> iou_mat(D, std::vector<float>(T, 0.f));
    for (size_t d = 0; d < D; ++d)
      for (size_t t = 0; t < T; ++t)
        iou_mat[d][t] = iou(dets[d], trks[t]);

    std::vector<int> det_assign(D, -1), trk_assign(T, -1);
    while (true) {
      float best = -1.f;
      size_t bi = 0, bj = 0;
      for (size_t d = 0; d < D; ++d)
        if (det_assign[d] == -1)
          for (size_t t = 0; t < T; ++t)
            if (trk_assign[t] == -1 && iou_mat[d][t] > best) {
              best = iou_mat[d][t];
              bi = d;
              bj = t;
            }
      if (best < iou_threshold_)
        break;
      det_assign[bi] = bj;
      trk_assign[bj] = bi;
    }

    std::vector<int> unmatched_dets, unmatched_trks;
    for (size_t d = 0; d < D; ++d)
      if (det_assign[d] == -1)
        unmatched_dets.push_back(d);
    for (size_t t = 0; t < T; ++t)
      if (trk_assign[t] == -1)
        unmatched_trks.push_back(t);

    for (size_t t = 0; t < T; ++t)
      if (trk_assign[t] != -1)
        trackers_[t]->update(dets[trk_assign[t]]);

    for (int idx : unmatched_dets)
      trackers_.push_back(std::make_unique<KalmanBoxTracker>(dets[idx]));

    std::vector<std::array<float, 5>> ret;
    for (int i = trackers_.size() - 1; i >= 0; --i) {
      auto box = trackers_[i]->get_state();
      if (trackers_[i]->time_since_update_ < 1 &&
          (trackers_[i]->hit_streak_ >= min_hits_ || frame_count_ <= min_hits_))
        ret.push_back({box.x, box.y, box.x + box.width, box.y + box.height,
                       static_cast<float>(trackers_[i]->id_)});
      if (trackers_[i]->time_since_update_ > max_age_)
        trackers_.erase(trackers_.begin() + i);
    }
    return ret;
  }

private:
  int max_age_;
  int min_hits_;
  float iou_threshold_;
  int frame_count_;
  std::vector<std::unique_ptr<KalmanBoxTracker>> trackers_;
};

static const std::map<int, std::string> CLASS_TO_COLOR{
    {0, "yellow"}, {1, "orange"}, {2, "orange"}, {3, "red"}, {4, "blue"}
};
static const std::vector<cv::Scalar> COLORS{
    cv::Scalar(0,255,255), // yellow
    cv::Scalar(0,165,255), // orange
    cv::Scalar(0,165,255), // orange
    cv::Scalar(0,0,255),   // red
    cv::Scalar(255,0,0)    // blue
};

class ConeDetectorNode : public rclcpp::Node {
public:
  ConeDetectorNode() : Node("cone_detector_cpp"), tracker_(5, 1, 0.3f) {
    declare_parameter<std::string>("onnx_path", "");
    get_parameter("onnx_path", onnx_model_path_);
    if (onnx_model_path_.empty()) {
      onnx_model_path_ =
          ament_index_cpp::get_package_share_directory("tensorrt_cone_detect_cpp") +
          "/resource/v11n_416x416.onnx";
      RCLCPP_WARN(get_logger(),
                  "Parameter 'onnx_path' not set. Using default: %s",
                  onnx_model_path_.c_str());
    }

    declare_parameter<std::vector<std::string>>("onnx_input_names", std::vector<std::string>{"images"});
    get_parameter("onnx_input_names", onnx_input_names_str_);
    for (auto &n : onnx_input_names_str_) onnx_input_names_ptr_.push_back(n.c_str());

    declare_parameter<std::vector<int64_t>>("onnx_input_dims", std::vector<int64_t>{1,3,416,416});
    get_parameter("onnx_input_dims", onnx_input_dims_);

    declare_parameter<std::vector<std::string>>("onnx_output_names", std::vector<std::string>{"output"});
    get_parameter("onnx_output_names", onnx_output_names_str_);
    for (auto &n : onnx_output_names_str_) onnx_output_names_ptr_.push_back(n.c_str());

    declare_parameter<int>("image_width", 416);
    declare_parameter<int>("image_height", 416);
    get_parameter("image_width", image_width_);
    get_parameter("image_height", image_height_);

    set_session_options();
    if (!load_model()) {
      RCLCPP_ERROR(get_logger(), "Failed to load model");
    }

    pub_overlay_ = create_publisher<sensor_msgs::msg::Image>("/camera/rgb/image_overlay", 10);
    pub_detections_ = create_publisher<oak_cone_detect_interfaces::msg::ConeArray2D>("/cone_detections_2d",10);
    pub_fps_ = create_publisher<std_msgs::msg::Float32>("/inference_fps", 10);

    sub_image_ = create_subscription<sensor_msgs::msg::Image>("/camera/rgb/image_raw", rclcpp::SensorDataQoS(),
                    std::bind(&ConeDetectorNode::image_cb, this, std::placeholders::_1));
  }

private:
  void set_session_options() {
    session_options_.SetInterOpNumThreads(1);
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
    int device_id = 0;
    try {
      Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(session_options_, device_id));
      RCLCPP_INFO(get_logger(), "TensorRT execution provider added");
    } catch (Ort::Exception &e) {
      RCLCPP_WARN(get_logger(), "TensorRT not available: %s", e.what());
    }
    try {
      Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(session_options_, device_id));
      RCLCPP_INFO(get_logger(), "CUDA execution provider added");
    } catch (Ort::Exception &e) {
      RCLCPP_WARN(get_logger(), "CUDA not available: %s", e.what());
    }
  }

  bool load_model() {
    try {
      session_ = Ort::Session(env_, onnx_model_path_.c_str(), session_options_);
      memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    } catch (Ort::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed loading model: %s", e.what());
      return false;
    }
    return true;
  }

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    rclcpp::Time start = now();
    cv::Mat img;
    if (msg->encoding == "bgr8") {
      img = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));
    } else if (msg->encoding == "rgb8") {
      img = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));
      cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    cv::Mat blob = cv::dnn::blobFromImage(img, 1/255.0, cv::Size(image_width_, image_height_), cv::Scalar(), true, false);

    std::vector<Ort::Value> input_tensors;
    size_t tensor_size = blob.total();
    input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info_, reinterpret_cast<float*>(blob.data), tensor_size, onnx_input_dims_.data(), onnx_input_dims_.size()));

    std::vector<Ort::Value> output_tensors;
    try {
      output_tensors = session_.Run(Ort::RunOptions{nullptr}, onnx_input_names_ptr_.data(), input_tensors.data(), input_tensors.size(), onnx_output_names_ptr_.data(), onnx_output_names_ptr_.size());
    } catch (Ort::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Inference error: %s", e.what());
      return;
    }

    auto overlay = img.clone();
    auto arr2d = oak_cone_detect_interfaces::msg::ConeArray2D();
    arr2d.header = msg->header;

    struct DetInfo {
      cv::Rect2f box;
      std::string label;
      float conf;
      std::string color;
    };
    std::vector<DetInfo> dets;
    std::vector<cv::Rect2f> boxes;

    for (auto &out : output_tensors) {
      const float *data = out.GetTensorData<float>();
      auto shape = out.GetTensorTypeAndShapeInfo().GetShape();
      size_t num = shape[1];
      size_t dims = shape[2];
      for (size_t i=0;i<num;++i) {
        const float *det = &data[i*dims];
        float x1=det[0], y1=det[1], x2=det[2], y2=det[3];
        float conf=det[4];
        int class_id = static_cast<int>(det[5]);
        if (conf < 0.5) continue;
        cv::rectangle(overlay, cv::Point(x1,y1), cv::Point(x2,y2), COLORS[class_id],2);
        cv::circle(overlay, cv::Point((x1+x2)/2,(y1+y2)/2),4,COLORS[class_id],-1);
        cv::putText(overlay, CLASS_TO_COLOR.at(class_id), cv::Point(x1,y1-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLORS[class_id],1);

        DetInfo d{cv::Rect2f(x1,y1,x2-x1,y2-y1), CLASS_TO_COLOR.at(class_id), conf, CLASS_TO_COLOR.at(class_id)};
        dets.push_back(d);
        boxes.push_back(d.box);
      }
    }

    auto tracks = tracker_.update(boxes);
    for (auto &tr : tracks) {
      float x1=tr[0], y1=tr[1], x2=tr[2], y2=tr[3];
      int tid = static_cast<int>(tr[4]);
      cv::Rect2f tbox(x1,y1,x2-x1,y2-y1);
      DetInfo *best=nullptr; float best_i=0.f;
      for(auto &d:dets){
        float iv=iou(tbox,d.box);
        if(iv>best_i){best_i=iv;best=&d;}
      }
      oak_cone_detect_interfaces::msg::Cone2D c;
      c.id = std::to_string(tid);
      if(best){
        c.label=best->label; c.conf=best->conf; c.color=best->color;
      }else{
        c.label=""; c.conf=0.0f; c.color="blue";
      }
      c.x=(x1+x2)/2.0f;
      c.y=(y1+y2)/2.0f;
      if(c.y > 0)
        arr2d.cones.push_back(c);
    }

    pub_detections_->publish(arr2d);

    auto overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg();
    pub_overlay_->publish(*overlay_msg);

    auto end = now();
    std_msgs::msg::Float32 fps_msg;
    fps_msg.data = 1.0f / (end - start).seconds();
    pub_fps_->publish(fps_msg);
  }

  std::string onnx_model_path_;
  std::vector<std::string> onnx_input_names_str_;
  std::vector<const char*> onnx_input_names_ptr_;
  std::vector<int64_t> onnx_input_dims_;
  std::vector<std::string> onnx_output_names_str_;
  std::vector<const char*> onnx_output_names_ptr_;

  int image_width_{416};
  int image_height_{416};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_overlay_;
  rclcpp::Publisher<oak_cone_detect_interfaces::msg::ConeArray2D>::SharedPtr pub_detections_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fps_;

  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "cone-detector"};
  Ort::SessionOptions session_options_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo memory_info_{nullptr};
  SortTracker tracker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

