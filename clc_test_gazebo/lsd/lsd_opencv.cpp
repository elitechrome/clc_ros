#include <iostream>
#include <opencv2/opencv.hpp>
extern "C"{
#include "lsd.h"
}

cv::Mat img;
int n_lines;
double* lines;
 
void change_th_lsd(int nfa, void* dummy)
{
    cv::Mat result = img.clone();
    for(int i = 0; i < n_lines; i++)
    {
        const double *line = &lines[i * 7];
        if(nfa < line[6])
        {
            const cv::Point p1(line[0], line[1]);
            const cv::Point p2(line[2], line[3]);
            cv::line(result, p1, p2, cv::Scalar(0, 0, 255));
        }
    }
    cv::imshow("result_image", result);
}
 
int main(int argc, char *argv[])
{
    //画像をグレースケールとして読み込み
    img = cv::imread(argv[1], 0);
 
    //LSD用画像に変換＞＜
    double *dat = new double[img.rows * img.cols];
    for(int y = 0; y < img.rows; y++)
        for(int x = 0; x < img.cols; x++)
            dat[y * img.cols + x] = img.at<unsigned char>(y, x);
 
    //LSD処理
    lines = lsd(&n_lines, dat, img.cols, img.rows);
 
    //しきい値の最大値と最小値をもってくる
    int max_NFA = 0;
    for(int i = 0; i < n_lines; i++)
        max_NFA = std::max(max_NFA, static_cast<int>(lines[i * 7 + 6]));
 
    //結果描画用画像
    cv::cvtColor(img, img, CV_GRAY2RGB);
 
    //結果表示用ウィンドウ
    cv::namedWindow("result_image");
    cv::createTrackbar("NFA", "result_image", NULL, max_NFA, change_th_lsd);
    cv::setTrackbarPos("NFA", "result_image", max_NFA);
 
    //結果表示
    cv::imshow("result_image", img);
    cv::waitKey(0);
}
