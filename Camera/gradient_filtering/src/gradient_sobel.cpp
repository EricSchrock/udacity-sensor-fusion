#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // create filter kernel
    float gauss_data[25] = {1, 4, 7, 4, 1,
                            4, 16, 26, 16, 4,
                            7, 26, 41, 26, 7,
                            4, 16, 26, 16, 4,
                            1, 4, 7, 4, 1};

    for (int i = 0; i < 25; i++)
    {
        gauss_data[i] /= 273;
    }

    cv::Mat kernel = cv::Mat(5, 5, CV_32F, gauss_data);

    // apply filter
    cv::Mat imgBlurred;
    cv::filter2D(imgGray, imgBlurred, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // create sobel x kernel
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2,
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    // apply filter
    cv::Mat result_x;
    cv::filter2D(imgBlurred, result_x, -1, kernel_x);

    // show result
    string windowName = "Sobel operator (x-direction)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, result_x);
    cv::waitKey(0); // wait for keyboard input before continuing

    // create sobel y kernel
    float sobel_y[9] = {-1, -2, -1,
                         0,  0,  0,
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply filter
    cv::Mat result_y;
    cv::filter2D(imgBlurred, result_y, -1, kernel_y);

    // show result
    windowName = "Sobel operator (y-direction)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, result_y);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    gradientSobel();
}