#include <tools.hpp>
#include "rclcpp/rclcpp.hpp"
namespace cango_master {
void calc_coordinate::load_semantic_map(const std::string& file_path) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    // 좌표 읽기
    if (config["nodes"]) {
      for (auto it = config["nodes"].begin(); it != config["nodes"].end();
           ++it) {
        map_nodes_.push_back({it->first.as<std::string>(),
                              it->second[0].as<double>(),
                              it->second[1].as<double>()});
      }
    }
    // 연결성 가져오기
    if (config["edges"]) {
      for (auto const& e : config["edges"]) {
        map_edges_.push_back({e["from"].as<std::string>(),
                              e["to"].as<std::string>()});
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("calc_coordinate"),
                 "fail to load semantic map: %s", e.what());
  }
}

void calc_coordinate::id2pcd(std::string id, Point& pt) {
  auto it = std::find_if(map_nodes_.begin(), map_nodes_.end(),
                         [&id](const NodeInfo& node) { return node.id == id; });

  if (it != map_nodes_.end()) {
    pt.x = it->x;
    pt.y = it->y;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("calc_coordinate"), "Cannot find node: %s", id.c_str());
  }
}

void calc_coordinate::pcd2id(Point pt, std::string& id_A, std::string& id_B) {
  double min_dist = 1e9;
  std::string best_start = "Unknown";
  std::string best_end = "Unknown";
  double best_t = 0.5;
  double best_edge_len = 0.0;

  // 1. 모든 에지를 순회하며 가장 가까운 선분 찾기
  for (const auto& edge : map_edges_) {
    auto A = get_node(edge.start);
    auto B = get_node(edge.end);

    double abx = B.x - A.x;
    double aby = B.y - A.y;
    double p_len_sq = abx * abx + aby * aby;
    if (p_len_sq == 0) continue;

    // 투영 비율 t 계산 및 클램핑 (0.0 ~ 1.0)
    double t = ((pt.x - A.x) * abx + (pt.y - A.y) * aby) / p_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    // 투영점과의 거리 계산
    double dist = std::hypot(pt.x - (A.x + t * abx), pt.y - (A.y + t * aby));
    if (dist < min_dist) {
      min_dist = dist;
      best_start = edge.start;
      best_end = edge.end;
      best_t = t;
      best_edge_len = std::sqrt(p_len_sq);
    }
  }

  // 2. 20cm(0.2m) 임계값 기반 ID 할당 로직
  if (best_start == "Unknown") {
    id_A = "Unknown";
    id_B = "Unknown";
    return;
  }

  double dist_from_start = best_t * best_edge_len;
  double dist_from_end = (1.0 - best_t) * best_edge_len;

  if (dist_from_start <= 0.2) {
    // 시작 노드 근처일 때 (A-A)
    id_A = best_start;
    id_B = best_start;
  } else if (dist_from_end <= 0.2) {
    // 끝 노드 근처일 때 (B-B)
    id_A = best_end;
    id_B = best_end;
  } else {
    // 복도 중간일 때 (A-B)
    id_A = best_start;
    id_B = best_end;
  }
}

}  // namespace cango_master
