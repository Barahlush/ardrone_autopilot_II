#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
struct Box {
    float left;
    float right;
    float top;
    float bottom;
};


struct CirclesMessage {
    std::vector<cv::RotatedRect> circles;
    Box box;
    std::vector<bool> inTheBox;
};

extern void processImage(cv::Mat&, CirclesMessage&);
