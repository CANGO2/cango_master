#include <string>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

template <class T>
static inline T clip(T now_val, T min_val, T max_val) {
  return max_val < now_val ? max_val : (min_val > now_val ? min_val : now_val);
}

// enum task_name {
//   None, operation, status, sound, llm, vision
// };
// enum status {
//   None, wait, moving, arrived, returning
// };


class calc_coordinate {
public:
    calc_coordinate(const std::string& file_path = "/home/lsk/colcon_ws/src/cango_master/config/symantic_transition.yaml") {
        load_semantic_map(file_path);
    }

    struct NodeInfo { std::string id; double x, y; };
    struct EdgeInfo { std::string start, end, label; };

    void load_semantic_map(const std::string& file_path);

    std::vector<NodeInfo> map_nodes_;
    std::vector<EdgeInfo> map_edges_;

    // 현재위치(PCD) -> Semantic ID 변환 
    void pcd2id(double rx, double ry, std::string &id_A, std::string &id_B); 
    
    // Semantic ID -> PCD 좌표 변환 
    void id2pcd(std::string id, double &x, double &y);

private:
    // 내부적으로 ID를 통해 노드 정보를 찾는 헬퍼 함수
    NodeInfo get_node(const std::string& id) {
        for (const auto& node : map_nodes_) {
            if (node.id == id) return node;
        }
        return {"Unknown", 0.0, 0.0};
    }
};

class pd_controller {
 private:
  double kp_, kd_;
  double prev_err_;

 public:
  inline void reset() { prev_err_ = 0.0; }
  inline void set_gain(const double kp, const double kd) { kp_ = kp, kd_ = kd; }
  double calculate(const double error) {
    double res = kp_ * error + kd_ * (error - prev_err_);
    prev_err_ = error;
    return res;
  }
};

class robot_command {
 public:
  double lin_vel = 0.0, ang_vel = 0.0;
};
 