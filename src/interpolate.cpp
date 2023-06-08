#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "matplotlibcpp.h"

void read_data(const std::string &file_name, std::vector<double> &xs, std::vector<double> &ys) {
    std::ifstream file(file_name);
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        boost::split(row, line, boost::is_any_of(","));
        xs.emplace_back(boost::lexical_cast<double>(row[0]));
        ys.emplace_back(boost::lexical_cast<double>(row[1]));
    }
}

template<typename T>
T lerp(T a, T b, T t) {
    return a + (b - a) * t;
}

int main(int argc, char **argv) {
    constexpr double kSegmentLength = 0.5;
    constexpr double kMathEps = 0.0001;


    std::string exec_path = argv[0];
    boost::filesystem::path path(exec_path);
    std::string data_path = path.parent_path().parent_path().append("map1_2023-4-18-16-19-36.csv").string();
    std::cout << data_path << std::endl;

    std::vector<double> x_s, y_s, acc_s;
    x_s.reserve(1000);
    y_s.reserve(1000);
    acc_s.reserve(1000);
    read_data(data_path, x_s, y_s);

    acc_s.emplace_back(0);
    for (size_t i = 1; i < x_s.size(); i++) {
        auto s = acc_s.back() + std::hypot(x_s[i] - x_s[i - 1], y_s[i] - y_s[i - 1]);
        acc_s.emplace_back(s);
    }

    auto total_length = acc_s.back();
    int segment_size = std::ceil(total_length / kSegmentLength);
    auto segment_slice = Eigen::VectorXd::LinSpaced(segment_size, 0, total_length);

    std::vector<double> x_s_sliced, y_s_sliced;
    x_s_sliced.resize(segment_slice.size());
    y_s_sliced.resize(segment_slice.size());
    auto lower = acc_s.begin();
    for (Eigen::Index i = 0; i < segment_slice.size(); i++) {
        auto target_s = segment_slice[i];
        auto upper = std::upper_bound(lower, acc_s.end(), target_s);

        if (upper == acc_s.end()) {
            x_s_sliced[i] = x_s.back();
            y_s_sliced[i] = y_s.back();
            break;
        }

        auto upper_index = std::distance(acc_s.begin(), upper);
        auto prev_upper = upper - 1;
        auto den = *upper - *prev_upper;
        auto t = den > kMathEps ? (target_s - *prev_upper) / den : 0;
        x_s_sliced[i] = lerp(x_s[upper_index - 1], x_s[upper_index], t);
        y_s_sliced[i] = lerp(y_s[upper_index - 1], y_s[upper_index], t);
        lower = prev_upper;
    }

    namespace plt = matplotlibcpp;

    plt::subplot(1, 2, 1);
    plt::plot(x_s, y_s, "b,");
    plt::axis("equal");
    plt::subplot(1, 2, 2);
    plt::plot(x_s_sliced, y_s_sliced, "r,");
    plt::axis("equal");
    plt::savefig("output.svg");

    return 0;
}
