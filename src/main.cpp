#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include "include/iterative_anchoring_smoother.h"
#include "include/reference_point.h"
#include "util/vec2d.h"
#include "matplotlibcpp.h"

using namespace apollo;
using namespace planning;

using apollo::common::math::Vec2d;

void read_data(const std::string &file_name, std::vector<double> &xs, 
                     std::vector<double> &ys, std::vector<double> &heading_s) {
    std::ifstream file(file_name);
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        boost::split(row, line, boost::is_any_of(","));
        xs.emplace_back(boost::lexical_cast<double>(row[0]));
        ys.emplace_back(boost::lexical_cast<double>(row[1]));
        heading_s.emplace_back(boost::lexical_cast<double>(row[2]));
    }
}

void write_to_csv(std::vector<double>const &smoothed_x, 
                  std::vector<double>const &smoothed_y,
                  std::vector<double>const &heading, 
                  std::vector<double>const &kappa, 
                  std::vector<double>const &dkappa)
{
  std::ofstream outFile;
  outFile.open("smoothed_path.csv", std::ios::out);
  outFile << "smoothed_x" << ',' << "smoothed_y" << ',' << "heading" << ',' 
          << "kappa" << ',' << "dkappa" << '\n';
  for (size_t i = 0; i < smoothed_x.size(); ++i)
  {
    outFile << smoothed_x.at(i) << ',' << smoothed_y.at(i) << ','
            << heading.at(i) << ',' << kappa.at(i) << ',' << dkappa.at(i) << '\n';
  }
  outFile.close();
}

template<typename T>
T lerp(T a, T b, T t) {
    return a + (b - a) * t;
}

int main(int argc, char **argv)
{
  std::string exec_path = argv[0];
  boost::filesystem::path path(exec_path);
  std::string data_path = path.parent_path().append("../HA_path.csv").string();
  std::cout << data_path << std::endl;

  std::vector<double> x_s, y_s, heading_s, acc_s;
  x_s.reserve(1000);
  y_s.reserve(1000);
  heading_s.reserve(1000);
  read_data(data_path, x_s, y_s, heading_s);

  Eigen::MatrixXd xWS(3, x_s.size());
  for (size_t i = 0; i < x_s.size(); i++) {
    xWS(0, i) = x_s[i];
    xWS(1, i) = y_s[i];
    xWS(2, i) = heading_s[i];
  }

  /* Add obstacles */
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec;
  std::vector<Vec2d> obs_v1, obs_v2;
  obs_v1.emplace_back(5, 4.8);
  obs_v1.emplace_back(5, 5.8);
  obs_v1.emplace_back(6, 5.8);
  obs_v1.emplace_back(6, 4.8);
  obs_v1.emplace_back(5, 4.8);
  obstacles_vertices_vec.emplace_back(obs_v1);

  obs_v2.emplace_back(-4, -1);
  obs_v2.emplace_back(-4, 0);
  obs_v2.emplace_back(-3, 0);
  obs_v2.emplace_back(-3, -1);
  obs_v2.emplace_back(-4, -1);
  obstacles_vertices_vec.emplace_back(obs_v2);

  /* Plot obstacles */
  std::vector<double> obs1_x, obs1_y;
  for (const auto& obstacles : obs_v1) {
    obs1_x.emplace_back(obstacles.x());
    obs1_y.emplace_back(obstacles.y());
  }

  std::vector<double> obs2_x, obs2_y;
  for (const auto& obstacles : obs_v2) {
    obs2_x.emplace_back(obstacles.x());
    obs2_y.emplace_back(obstacles.y());
  }

  double init_a = 0.0, init_v = 0.0;
  std::vector<common::TrajectoryPoint> trajectory_points;
  DiscretizedTrajectory discretized_trajectory(trajectory_points);

  PlannerOpenSpaceConfig planner_open_space_config;
  auto smoother = std::make_shared<IterativeAnchoringSmoother>(planner_open_space_config);  
  
  smoother->Smooth(xWS, init_a, init_v, obstacles_vertices_vec, &discretized_trajectory);

  std::vector<double> smoothed_x, smoothed_y, smoothed_heading, 
                      smoothed_kappa, smoothed_dkappa, 
                      smoothed_v,smoothed_a;
  
  for (const auto& point : discretized_trajectory) {
    smoothed_x.emplace_back(point.path_point().x());
    smoothed_y.emplace_back(point.path_point().y());
    smoothed_heading.emplace_back(point.path_point().theta());
    smoothed_kappa.emplace_back(point.path_point().kappa());
    smoothed_dkappa.emplace_back(point.path_point().dkappa());
    smoothed_v.emplace_back(point.v());
    smoothed_a.emplace_back(point.a());
  }

  std::vector<std::vector<double>> collided_x, collided_y;
  std::vector<double> vertices_x, vertices_y;
  for (const auto& ego : smoother->collided_ego_box_) {
    for (const auto& vertices : ego) {
      vertices_x.emplace_back(vertices.x());
      vertices_y.emplace_back(vertices.y());
    }
    vertices_x.emplace_back(ego.front().x());
    vertices_y.emplace_back(ego.front().y());

    collided_x.emplace_back(vertices_x);
    collided_y.emplace_back(vertices_y);
  }

  namespace plt = matplotlibcpp;

  plt::named_plot("Raw", x_s, y_s, "b.");
  plt::named_plot("Smoothed", smoothed_x, smoothed_y, "r.");

  // plt::named_plot("kappa", smoothed_kappa, ".");
  // plt::named_plot("dkappa", smoothed_dkappa, ".");

  // plt::named_plot("vel", smoothed_v, ".");
  // plt::named_plot("acc", smoothed_a, ".");

  plt::named_plot("obs1", obs1_x, obs1_y);
  plt::named_plot("obs2", obs2_x, obs2_y);

  // for (size_t i = 0; i < collided_x.size(); i++) {
  //   plt::plot(collided_x.at(i), collided_y.at(i), "c");
  // }
  plt::axis("equal");
  plt::xlabel("x/m");
  plt::ylabel("y/m");
  plt::title("Path Compare");
  plt::legend();
  plt::save("Compare.svg");
  plt::show();


  return 0;
}