#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库
#include<opencv2/core/core.hpp>  //OpenCV2标准头文件
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>    
#include <vector>
#include <cv.h>  
#include <algorithm>

#include "gige_cap/gige_config.h"
#include "gige_cap/SetParam.h"
using namespace gige;
using namespace gige_cap;

static const float PI = 3.1415926535f;
static const float eps = 0.001f;

using namespace std;
using namespace cv;

class Image_Thresholder
{
    private:
        float curvature[150][2];
        float ball_xcordinate;
        float ball_ycordinate;
        float theta;

    public:
	struct timeval start;
	struct timeval end;	

    public:
	void Converter(const cv::Mat& src, cv::Mat& dst);
        void Threshold_Ball(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i);
	float Max_Area(Mat& src,Mat& dst,char type);
        float Calculate_Distance(float x, float y);
        void curvature_lookup_table();
        float get_theta();
};
	
