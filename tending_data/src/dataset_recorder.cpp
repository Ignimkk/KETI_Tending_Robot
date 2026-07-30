// Tending 데이터 기록 노드
//
// inspection_sample 토픽(tending_interfaces/msg/InspectionSample)을 구독하여
// 검사 뷰마다 JSON 사이드카를 이미지 옆에 저장하고, manifest.jsonl 에 한 줄씩 append 한다.
//
// 동기화 방식(P0, stop-and-shoot): inspection_manager 가 로봇이 정지한 촬영 순간의
// 관절값/TCP 포즈/각도/이미지 경로를 InspectionSample 하나에 담아 게시하므로,
// 본 노드는 그 스냅샷을 그대로 영속화만 한다(결정적, message_filters 불필요).
// 연속 모션(P1)으로 확장 시 message_filters ApproximateTime 동기로 대체 가능.

#include <fstream>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tending_interfaces/msg/inspection_sample.hpp"

namespace fs = std::filesystem;
using InspectionSample = tending_interfaces::msg::InspectionSample;

class DatasetRecorder : public rclcpp::Node
{
public:
  DatasetRecorder()
  : rclcpp::Node("dataset_recorder")
  {
    output_dir_ = this->declare_parameter<std::string>("output_dir", "/tmp/tending_dataset");

    sub_ = this->create_subscription<InspectionSample>(
      "inspection_sample", rclcpp::QoS(50),
      std::bind(&DatasetRecorder::on_sample, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "dataset_recorder 시작. 저장 위치: %s", output_dir_.c_str());
  }

private:
  static std::string json_num(double v)
  {
    std::ostringstream oss;
    oss << std::setprecision(9) << v;
    return oss.str();
  }

  std::string sample_to_json(const InspectionSample & m) const
  {
    std::ostringstream j;
    j << "{";
    j << "\"view_index\":" << m.view_index << ",";
    j << "\"angle\":" << json_num(m.angle) << ",";
    j << "\"stamp\":" << json_num(
      static_cast<double>(m.header.stamp.sec) + static_cast<double>(m.header.stamp.nanosec) * 1e-9) << ",";
    j << "\"frame_id\":\"" << m.header.frame_id << "\",";
    j << "\"image_path\":\"" << m.image_path << "\",";
    j << "\"joint_pos\":[";
    for (size_t i = 0; i < m.joint_pos.size(); ++i) {
      j << json_num(m.joint_pos[i]);
      if (i + 1 < m.joint_pos.size()) j << ",";
    }
    j << "],";
    const auto & p = m.tcp_pose.position;
    const auto & o = m.tcp_pose.orientation;
    j << "\"tcp_position\":[" << json_num(p.x) << "," << json_num(p.y) << "," << json_num(p.z) << "],";
    j << "\"tcp_orientation\":[" << json_num(o.x) << "," << json_num(o.y) << ","
      << json_num(o.z) << "," << json_num(o.w) << "]";
    j << "}";
    return j.str();
  }

  void on_sample(const InspectionSample::SharedPtr m)
  {
    try {
      // 이미지 경로가 있으면 그 폴더에, 없으면 output_dir 에 저장.
      fs::path base = m->image_path.empty()
        ? fs::path(output_dir_)
        : fs::path(m->image_path).parent_path();
      fs::create_directories(base);

      const std::string json = sample_to_json(*m);

      char name[64];
      std::snprintf(name, sizeof(name), "view_%04u.json", m->view_index);
      std::ofstream side(base / name, std::ios::trunc);
      side << json << "\n";

      std::ofstream manifest(base / "manifest.jsonl", std::ios::app);
      manifest << json << "\n";

      RCLCPP_INFO(get_logger(), "[view %u] 메타 저장 (angle=%.3f rad)", m->view_index, m->angle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "메타 저장 예외: %s", e.what());
    }
  }

  std::string output_dir_;
  rclcpp::Subscription<InspectionSample>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatasetRecorder>());
  rclcpp::shutdown();
  return 0;
}
