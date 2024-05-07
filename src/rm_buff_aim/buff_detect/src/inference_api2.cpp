/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-21 16:24:35
 * @LastEditTime: 2023-06-01 17:08:34
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/inference/inference_api2.cpp
 */
#include "buff_detect/inference_api2.hpp"


namespace buff_detector
{
    BuffDetector::BuffDetector()
    {
    }

    BuffDetector::~BuffDetector()
    {
    }

    bool BuffDetector::initModel(std::string path)
    {
        auto compile_model = core.compile_model("/home/dhu/DIODE_buff/src/buff_detect/model/best.xml", "GPU");
        infer_request = compile_model.create_infer_request();
        input_port = compile_model.input();

        return true;
    }

    bool BuffDetector::detect(cv::Mat &src, std::vector<BuffObject>& objects)
    {
        if (src.empty())
        {
            cout << "[DETECT] ERROR: 传入了空的src";
            return false;
        }


        cv::Mat letterbox_img = letterbox(src);
        float scale = letterbox_img.size[0] / 640.0;

        cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0/255, cv::Size(640, 640), cv::Scalar(), true);

        ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));

        infer_request.set_input_tensor(input_tensor);

        infer_request.infer();

        auto output0 = infer_request.get_output_tensor(0);

        //后处理
        cv::Mat output_buffer(output0.get_shape()[1], output0.get_shape()[2], CV_32F, output0.data<float>());

        cv::transpose(output_buffer, output_buffer);
        
        float score_thre = 0.8;
        float nms_thre = 0.5;

        //类别id 得分 矩形框 关键点
        std::vector<int> class_ids;     //0红色 1蓝色
        std::vector<float> class_scores;
        std::vector<cv::Rect> boxes;
        std::vector<cv::Mat> mask_confs;

        for (int i = 0; i < output_buffer.rows; ++i){
            cv::Mat scores = output_buffer.row(i).colRange(4, 6);
            cv::Point class_id;
            double maxClassScore;
            cv::minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);
            
            if (maxClassScore > score_thre){
                class_scores.push_back(maxClassScore);
                class_ids.push_back(class_id.x);
                float cx = output_buffer.at<float>(i, 0);
                float cy = output_buffer.at<float>(i, 1);
                float w = output_buffer.at<float>(i, 2);
                float h = output_buffer.at<float>(i, 3);

                int left = int((cx - 0.5*w) * scale);
                int top = int((cy - 0.5*h) * scale);
                int width = int(w * scale);
                int height = int(h * scale);


                cv::Mat mask_conf = output_buffer.row(i).colRange(6, 21);

                for (int j = 0; j < mask_conf.cols; j += 3){
                    cv::Mat col1 = mask_conf.col(j); 
                    col1 *= scale;
                    cv::Mat col2 = mask_conf.col(j+1); 
                    col2 *= scale;
                }

                mask_confs.push_back(mask_conf);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }

        std::vector<int> indices;
        //nms极大值抑制
        cv::dnn::NMSBoxes(boxes, class_scores, score_thre, nms_thre, indices);

        if (indices.size() == 0){
            return false;
        }
        
        BuffObject obj;
        obj.rect = boxes[indices[0]];
        obj.prob = class_scores[indices[0]];
        obj.cls = 0;

        obj.color = class_ids[indices[0]] == 0 ? 1 : 0;
        cv::Mat points = mask_confs[indices[0]];
        obj.apex[0] = cv::Point2f(int(points.at<_Float32>(0)), int(points.at<_Float32>(1)));
        obj.apex[1] = cv::Point2f(int(points.at<_Float32>(3)), int(points.at<_Float32>(4)));
        obj.apex[2] = cv::Point2f(int(points.at<_Float32>(6)), int(points.at<_Float32>(7)));
        obj.apex[3] = cv::Point2f(int(points.at<_Float32>(9)), int(points.at<_Float32>(10)));
        obj.apex[4] = cv::Point2f(int(points.at<_Float32>(12)), int(points.at<_Float32>(13)));

        objects.resize(1);
        objects[0] = obj;

        return true;
    }

    cv::Mat BuffDetector::letterbox(const cv::Mat src)
    {
        int col = src.cols;
        int row = src.rows;
        int _max = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        src.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }

} //namespace buff_detector




