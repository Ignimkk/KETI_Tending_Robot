// Tending 카메라 노드
//
// capture_image 서비스를 제공한다.
//  - use_stub=true (기본, 카메라 부착 전): 합성 이미지(PPM)를 output_dir 아래 저장하고
//    경로/타임스탬프를 반환한다. 실제 하드웨어 없이 전체 파이프라인을 검증하기 위한 목 구현.
//  - use_stub=false (카메라 부착 후, P1): 실제 산업용 카메라 드라이버로 교체.
//    본 노드의 서비스 인터페이스(CaptureImage)는 그대로 유지하여 상위 계층을 바꾸지 않는다.

#include <cstdint>
#include <fstream>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tending_interfaces/srv/capture_image.hpp"

namespace fs = std::filesystem;
using CaptureImage = tending_interfaces::srv::CaptureImage;

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : rclcpp::Node("camera_node")
  {
    use_stub_ = this->declare_parameter<bool>("use_stub", true);
    output_dir_ = this->declare_parameter<std::string>("output_dir", "/tmp/tending_dataset");
    width_ = this->declare_parameter<int>("image_width", 640);
    height_ = this->declare_parameter<int>("image_height", 480);

    service_ = this->create_service<CaptureImage>(
      "capture_image",
      std::bind(&CameraNode::handle_capture, this, std::placeholders::_1, std::placeholders::_2));

    if (use_stub_) {
      RCLCPP_WARN(get_logger(),
        "tending_camera STUB 모드로 동작합니다 (카메라 미부착). 합성 이미지를 '%s' 아래에 저장합니다.",
        output_dir_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "tending_camera 실카메라 모드 (미구현 - P1). 현재는 stub 로 폴백합니다.");
    }
  }

private:
  void handle_capture(
    const std::shared_ptr<CaptureImage::Request> req,
    std::shared_ptr<CaptureImage::Response> res)
  {
    res->stamp = this->now();

    try {
      fs::path dir = fs::path(output_dir_) / req->label;
      fs::create_directories(dir);

      char name[64];
      std::snprintf(name, sizeof(name), "view_%04u.ppm", req->view_index);
      fs::path file = dir / name;

      if (!write_stub_ppm(file.string(), req->view_index)) {
        res->ok = false;
        res->image_path = "";
        RCLCPP_ERROR(get_logger(), "이미지 저장 실패: %s", file.string().c_str());
        return;
      }

      res->ok = true;
      res->image_path = file.string();
      RCLCPP_INFO(get_logger(), "[view %u] 이미지 저장: %s", req->view_index, file.string().c_str());
    } catch (const std::exception & e) {
      res->ok = false;
      res->image_path = "";
      RCLCPP_ERROR(get_logger(), "capture 예외: %s", e.what());
    }
  }

  // 뷰 인덱스에 따라 색이 변하는 간단한 그라디언트 합성 PPM(P6) 이미지.
  bool write_stub_ppm(const std::string & path, uint32_t view_index)
  {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
      return false;
    }
    ofs << "P6\n" << width_ << " " << height_ << "\n255\n";
    const uint8_t phase = static_cast<uint8_t>((view_index * 37u) & 0xFF);
    std::vector<uint8_t> row(static_cast<size_t>(width_) * 3u);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        row[x * 3 + 0] = static_cast<uint8_t>((x * 255) / std::max(1, width_ - 1));
        row[x * 3 + 1] = static_cast<uint8_t>((y * 255) / std::max(1, height_ - 1));
        row[x * 3 + 2] = phase;
      }
      ofs.write(reinterpret_cast<const char *>(row.data()), static_cast<std::streamsize>(row.size()));
    }
    return ofs.good();
  }

  bool use_stub_;
  std::string output_dir_;
  int width_;
  int height_;
  rclcpp::Service<CaptureImage>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
