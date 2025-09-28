#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 1. Load image
    cv::Mat img = cv::imread("laser_sample.jpg");
    if (img.empty()) {
        std::cerr << "Error: cannot load image" << std::endl;
        return -1;
    }

    // 2. Apply Gaussian blur to reduce noise
    cv::Mat smooth;
    cv::GaussianBlur(img, smooth, cv::Size(5,5), 0);

    // 3. Compute red response v = r - (g+b)/2
    cv::Mat response(img.rows, img.cols, CV_32F, cv::Scalar(0));
    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            cv::Vec3b pix = smooth.at<cv::Vec3b>(y,x);
            int r = pix[2], g = pix[1], b = pix[0]; // OpenCV stores as BGR
            float v = r - (g + b)/2.0f;
            response.at<float>(y,x) = std::max(v, 0.0f);
        }
    }

    // 4. Compute global threshold T = 0.8 * avgMax
    std::vector<float> rowMax(img.rows, 0.0f);
    float sumMax = 0.0f;
    for (int y = 0; y < img.rows; ++y) {
        double minv, maxv;
        cv::minMaxLoc(response.row(y), &minv, &maxv);
        rowMax[y] = (float)maxv;
        sumMax += (float)maxv;
    }
    float avgMax = sumMax / img.rows;
    float T = 0.8f * avgMax;

    // 5. For each row, keep only the peak if above threshold
    cv::Mat output(img.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < img.rows; ++y) {
        cv::Mat row = response.row(y);
        cv::Point maxLoc;
        double maxVal;
        cv::minMaxLoc(row, nullptr, &maxVal, nullptr, &maxLoc);
        if (maxVal >= T) {
            output.at<uchar>(y, maxLoc.x) = 255; // mark white pixel
        }
    }

    // 6. Display results
    cv::imshow("Input", img);
    cv::imshow("Laser Detection", output);
    cv::waitKey(0);

    return 0;
}
