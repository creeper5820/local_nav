#include "filter.hpp"
#include "../utility/param.hpp"

#include <opencv2/opencv.hpp>

void filter::handle(std::vector<type::Node>& data)
{

    auto mat = cv::Mat(int(param::grid_width), int(param::grid_width), CV_8UC1);

    for (const auto node : data)
        mat.at<int8_t>(node.x, node.y) = node.value;

    auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(mat, mat, element, cv::Point(-1, -1), 6);

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);

    for (auto& node : data)
        node.value = mat.at<int8_t>(node.x, node.y);
}
