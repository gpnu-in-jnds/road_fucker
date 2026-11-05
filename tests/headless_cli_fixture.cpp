#include "test_support.hpp"

#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <iostream>

int main() {
  try {
    using namespace road_finder::test;
    for (const auto& entry : {std::make_pair("straight", 0), std::make_pair("end", 300),
                              std::make_pair("close_end", 320)})
      require(cv::imwrite(std::string("cli_") + entry.first + ".png", image(road(entry.second))),
              "cannot write CLI fixture");
    require(cv::imwrite("cli_unknown.png", cv::Mat(360,640,CV_8UC3,cv::Scalar(160,160,160))),
            "cannot write lost-road fixture");
    cv::Mat outlier = road();
    outlier.rowRange(330,360).setTo(0);
    outlier(cv::Rect(500,330,81,30)).setTo(255);
    outlier.rowRange(326,334).setTo(0);  // Avoid blur mixing distant patches across strip edges.
    require(cv::imwrite("cli_filtered.png", image(outlier)), "cannot write filtered fixture");
    std::ofstream file("cli_config.yaml");
    file << "%YAML:1.0\n---\ninput_width: 640\ninput_height: 360\nprocess_width: 320\n"
            "roi: [0,180,640,180]\nroad_end_roi: [280,240,80,50]\n";
    require(static_cast<bool>(file), "cannot write CLI config");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
