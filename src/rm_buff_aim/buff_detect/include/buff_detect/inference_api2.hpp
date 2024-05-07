/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-21 16:24:35
 * @LastEditTime: 2022-12-27 19:01:00
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/inference/inference_api2.hpp
 */
#ifndef INFERENCE_API2_HPP_
#define INFERENCE_API2_HPP_

//C++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <openvino/openvino.hpp>
// #include <format_reader_ptr.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

// #include <fftw3.h>
// #include <fmt/color.h>

#include "global_user/global_user.hpp"

using namespace global_user;
namespace buff_detector
{
    struct BuffObject : Object
    {
        cv::Point2f apex[5];
    };

    class BuffDetector
    {
    public:
        BuffDetector();
        ~BuffDetector();
        
        bool detect(cv::Mat &src, std::vector<BuffObject>& objects);
        bool initModel(std::string path);
    private:

        cv::Mat letterbox(const cv::Mat src);

        ov::Core core;
        ov::InferRequest infer_request;
        ov::Output<const ov::Node> input_port;
    };
} // namespace buff_detector

#endif