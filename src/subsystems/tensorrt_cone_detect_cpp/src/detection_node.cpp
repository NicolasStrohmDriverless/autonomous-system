#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "oak_cone_detect_interfaces/msg/cone2_d.hpp"
#include "oak_cone_detect_interfaces/msg/cone_array2_d.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

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
  ConeDetectorNode() : Node("cone_detector_cpp") {
    declare_parameter<std::string>("onnx_path", "");
    get_parameter("onnx_path", onnx_model_path_);

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

        oak_cone_detect_interfaces::msg::Cone2D c;
        c.id = std::to_string(arr2d.cones.size());
        c.label = CLASS_TO_COLOR.at(class_id);
        c.conf = conf;
        c.x = (x1+x2)/2;
        c.y = (y1+y2)/2;
        c.color = CLASS_TO_COLOR.at(class_id);
        arr2d.cones.push_back(c);
      }
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

