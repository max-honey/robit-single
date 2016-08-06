#include "msg_transport.h"
//#define GIGN

RGB_Transport::RGB_Transport():it_(nh_),max_h(360),min_h(0),max_s(255),min_s(0),max_i(255),min_i(0),count(0)//构造函数
{
    //初始化输入输出窗口  
    //cv::namedWindow(INPUT);  
    //cv::namedWindow(OUTPUT); 
    thresholder.start.tv_sec=0;
    thresholder.end.tv_sec=0;
    thresholder.curvature_lookup_table(); 
    Create_Trackbar();
}   

RGB_Transport::~RGB_Transport() 
{  
    //销毁输入输出窗口 
    //cv::destroyWindow(INPUT);  
    //cv::destroyWindow(OUTPUT);  
}

void RGB_Transport::Create_Trackbar()
{
    namedWindow("trackbar", 1);
    createTrackbar("h_max", "trackbar", &max_h, 360, 0);
    createTrackbar("h_min", "trackbar", &min_h, 360, 0);
    createTrackbar("s_max", "trackbar", &max_s, 255, 0);
    createTrackbar("s_min", "trackbar", &min_s, 255, 0);
    createTrackbar("i_max", "trackbar", &max_i, 255, 0);
    createTrackbar("i_min", "trackbar", &min_i, 255, 0);
}

void RGB_Transport::Msg_Subsribe()
{
    //#ifdef GIGN
	image_sub_ = it_.subscribe("/GigE/image", 1, &RGB_Transport::convert_callback, this); //定义图象接受器，订阅话题是“image”
    //#else
	//cout << "1" << endl;
        //image_sub_ = it_.subscribe("imagesend", 1, &RGB_Transport::convert_callback, this); //定义图象接受器，订阅话题是“imagesend”
    //#endif
}

void RGB_Transport::Msg_Publisher()
{
  location_pub=nh_.advertise<Location>("location",10);
  //location_pub = it_.advertise<Location>("location", 10);
}

void RGB_Transport::convert_callback(const sensor_msgs::ImageConstPtr& msg)   
{   
    count+=1;
    gettimeofday(&thresholder.end,NULL);
    if(thresholder.end.tv_sec-thresholder.start.tv_sec>=1)
    {
	cout << "fps= " << count << endl;
        thresholder.start.tv_sec=thresholder.end.tv_sec;
        count=0;
    }	 
    //cout << "transport" << endl;
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
    try  
    {  
        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中图象信息提取，生成新cv类型的图象复制给CvImage指针  
        if(cv_ptr==NULL)
        {
            cout << "NULL" <<endl;
	}
    }  
    catch(cv_bridge::Exception& e)  //异常处理  
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    }   
    src=cv_ptr->image.clone();
    thresholder.Converter(src,dst_hsi);
    dst_ball = dst_hsi.clone(); 
    dst_obstacle = dst_hsi.clone();
    dst_field = dst_hsi.clone();
    dst_line = dst_hsi.clone();
    thresholder.Threshold_Ball(src,dst_ball,max_h,min_h,max_s,min_s,max_i,min_i);
    msg_location.distance=thresholder.Max_Area(src,dst_ball,type);
    msg_location.theta=thresholder.get_theta(); 
    cout << "reaa_distance: " << msg_location.distance << endl;
    cout << "theta:" << msg_location.theta << endl;
    location_pub.publish(msg_location);
    //cout << max_h << " " << min_h << endl;
    waitKey(1);
}

void RGB_Transport::set_value(int maxh, int maxs, int maxi, int minh, int mins, int mini, char object)
{
    max_h=maxh;
    max_s=maxs;
    max_i=maxi;
    min_h=minh;
    min_s=mins;
    min_i=mini;
    type=object;
}

Mat RGB_Transport::get_src()
{
    //cout << "get_src" << endl;
    //imshow("output_src_get",src);
    //waitKey(1);
    return src;
}

int RGB_Transport::get_max_h()
{
    return max_h;
}

int RGB_Transport::get_min_h()
{
    return min_h;
}

int RGB_Transport::get_max_s()
{
    return max_s;
}

int RGB_Transport::get_min_s()
{
    return min_s;
}

int RGB_Transport::get_max_i()
{
    return max_i;
}

int RGB_Transport::get_min_i()
{
    return min_i;
}




