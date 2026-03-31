#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <string>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace cango_master {

template <class T>
static inline T clip(T now_val, T min_val, T max_val) {
  return max_val < now_val ? max_val : (min_val > now_val ? min_val : now_val);
}

struct Point { double x; double y; };

class calc_coordinate {
public:
    calc_coordinate(const std::string& file_path = "/home/lsk/colcon_ws/src/cango_master/config/symantic_transition.yaml") {
        load_semantic_map(file_path);
    }

    struct NodeInfo { std::string id; double x, y; };
    struct EdgeInfo { std::string start, end, label; };

    void load_semantic_map(const std::string& file_path);
    void serch_node(std::string id, Point &pt);
    
    std::vector<NodeInfo> map_nodes_;
    std::vector<EdgeInfo> map_edges_;

    void pcd2id(Point pt, std::string &id_A, std::string &id_B); 
    void id2pcd(std::string id, Point &pt);

private:
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
 

}  // namespace cango_master
#endif  // TOOLS_HPP_