#include "Thresholder.h"
#include <math.h>

void Image_Thresholder::Converter(const cv::Mat& src, cv::Mat& dst)
{
    dst = cv::Mat_<Vec3f>(src.rows, src.cols);
    int i, j;
    float rv, gv, bv;
    int ri, gi, bi;
    double hv, sv, iv, minv;
    for (i = 0; i < src.rows; i++)
    {
	const uchar* datIn = src.ptr<const uchar>(i);
	float* datOut = dst.ptr<float>(i);
	for (j = 0; j < src.cols; j++)
	{
	    //转化成0.0 ~ 1.0
	    ri = *(datIn++);
            gi = *(datIn++);
	    bi = *(datIn++);
	    rv = ri / 255.0f;
	    gv = gi / 255.0f;
            bv = bi / 255.0f;
	    minv = min(rv, min(gv, bv));
	    iv = (rv + gv + bv) / 3.0f;
	    sv = 1.0f - minv / (iv + eps);
	    //这里h采用近似算法
	    if (minv == rv)
	    {
		if ((gv + bv - 2 * rv) <= 0.001)
		{
		    hv = 0.0;
		}
		if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		{
	            hv = 0;
		}
		else
		{
		    hv = (bv - rv) / 3 / (gv + bv - 2 * rv) + 1.0 / 3.0;
		}
	    }
	    else if (minv == gv)
	    {
		if ((rv + bv - 2 * gv) <= 0.001)
		{
		    hv = 0.0;
		}
		if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		{
		    hv = 0;
		}
		else
		{
		    hv = (rv - gv) / 3 / (rv + bv - 2 * gv) + 2.0 / 3.0;
		}
	    }
	    else
	    {
		if ((rv + gv - 2 * bv) <= 0.001)
		{
		    hv = 0.0;
		}
		else
		{
		    if (rv <= 0.001&&gv <= 0.001&&bv <= 0.001)
		    {
			hv = 0;
		    }
		    hv = (gv - bv) / 3 / (rv + gv - 2 * bv) + 0.0001;
		}
	    }
	    //输出
	    *(datOut++) = hv*360;
	    *(datOut++) = sv*255;
	    *(datOut++) = iv*255;
	}
    }
    //cout << hv << " " << sv << " " << iv << endl;
    //cv::imshow("output_converter_hsi", dst);  
    cv::waitKey(5);
}

void Image_Thresholder::Threshold_Ball(const cv::Mat& src, cv::Mat& dst, int max_h, int min_h, int max_s, int min_s, int max_i, int min_i)
{
    float hv, sv, iv;
    int i, j;
    for (i = 0; i < src.rows; i++)
    {
	float* datIn = dst.ptr<float>(i);
	float* datOut = dst.ptr<float>(i);
	for (j = 0; j < src.cols; j++)
	{		
	    hv = *(datIn++);
	    sv = *(datIn++);
	    iv = *(datIn++);
	    int a = src.at<Vec3b>(i, j)[0];
	    int b = src.at<Vec3b>(i, j)[1];
            int c = src.at<Vec3b>(i, j)[2];
	    if (a == b||b == c)
	    {
		hv = 0;
	    }
	    if (hv >= min_h&&hv <= max_h&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
	    {
		hv = 1.0;
		sv = 1.0;
		iv = 1.0;
	    }
            else if (min_h>max_h&&(hv <= min_h||hv >= max_h)&&sv >= min_s&&sv <= max_s&& iv >= min_i&&iv <= max_i)
	    {
		hv = 1.0;
		sv = 1.0;
		iv = 1.0;
	    }
 
	    else
	    {
		hv = 0.0;
		sv = 0.0;
		iv = 0.0;
		
	    }
	    *(datOut++) = hv;
	    *(datOut++) = sv;
	    *(datOut++) = iv;
         }
    }
    //cv::imshow("threshold_ball", dst);  
    cv::waitKey(1);   
}

float Image_Thresholder::Max_Area(Mat& src,Mat& dst,char type)
{
    Mat aChannels[3];
    float real_distance=0;
    vector<vector <cv::Point> > contours;
    vector<Mat> vec;
    split(dst, vec);
    cv::Mat result1, result2;
    //cv::imshow("1", vec[0]);
    //cv::imshow("2", vec[1]);
    //cv::imshow("3", vec[2]);
    // 查找轮廓，对应连通域  
    int i = vec[0].type();
    vec[0].convertTo(vec[0], CV_8U);
    cv::findContours(vec[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // 寻找最大连通域
    dst.copyTo(result1);
    dst.copyTo(result2);
    if(type=='B')
    {  
        double maxArea = 0;
        vector<cv::Point> maxContour;
        for (size_t i = 0; i < contours.size(); i++)
        {
	    double area = cv::contourArea(contours[i]);
            if (area > maxArea)
            {
	        maxArea = area;
	        maxContour = contours[i];
            }
        }
        // 将轮廓转为矩形框  
        if (maxContour.empty())
        {
	    //cout << "ball did not detected" << endl;
	    return 0;
        }
        cv::Rect maxRect = cv::boundingRect(maxContour);
	ball_xcordinate = maxRect.x + 0.5*maxRect.width;
	ball_ycordinate = maxRect.y - 0.5*maxRect.height;
	cv::rectangle(result2, maxRect, cv::Scalar(255));
        cv::imshow("largest region", result2);
        cv::waitKey(1);
        //cout << ball_xcordinate << " " << ball_ycordinate << endl;
	real_distance=Calculate_Distance(ball_xcordinate, ball_ycordinate);
        return real_distance;
    }
    else
    {
        for (size_t i = 0; i < contours.size(); i++)
        {
	    cv::Rect r = cv::boundingRect(contours[i]);
	    cv::rectangle(result1, r, cv::Scalar(255));
        }
        cv::imshow("all regions", result1);
        cv::waitKey(1);
    }
    return 0;
}

float Image_Thresholder::Calculate_Distance(float x, float y)
{
	
	x -= 320.0;
	y -= 240.0;
	char s[100];
	int i;
	int distance = sqrt(x*x + y*y);
	float real_distance = 0.0;
	//cout << "Image_Distance:" <<distance << endl;
	if (distance <= 50)
	{
		//cout << "within the agent" << endl;
	}
	else if (distance >= 200)
	{
		//cout << "data can not be trusted" << endl;
	}
	else
	{
		//cout << x << " " << y << endl;
		for (int i = 0; i < 200; i++)
		{
			if (distance == curvature[i][0])
			{
				real_distance = curvature[i][1];
				cout << "real distance：" << real_distance << endl;
				break;
			}
		}
	}
        //cout << distance << endl;
        theta=atan2(x,y)/2/PI*360;
        if(theta<0)
        {
	    theta=360+theta;
	}
        cout << theta << endl;
        return real_distance;
}

void Image_Thresholder::curvature_lookup_table()
{
	char s[100];
	float i;
	int j, k = 0;
	FILE *fp;
	fp = fopen("/home/robit/catkin_ws/qulv.txt", "r"); /*打开文字文件只读*/
	fgets(s, 7, fp); /*从文件中读取23个字符*/
	//cout << s << endl;
	for (j = 0; j < 152; j++)
	{
		k = 0;
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k] = i;
		//printf("%f", curvature[j][k]);
		fgetc(fp); /*读取一个字符*/
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k+1] = i;
		//printf("%f\n", curvature[j][k+1]);

	}
	fclose(fp);
}

float Image_Thresholder::get_theta()
{
    return theta;
}
















