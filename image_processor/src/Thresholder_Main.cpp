#include "msg_transport.h"

using namespace gige;
using namespace gige_cap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RGB");
    RGB_Transport Image;
    int max_h=360, max_s=255, max_i=255;
    int min_h=0, min_s=0, min_i=0;
    char type;
    cout << "Please input the object you thresholded: " ;
    cout << " 'B' represents Ball, 'L' represents Line, 'F' represents Field, 'O' represents   Obstacle " << endl;
    cin >> type ;
    Image.set_value(max_h, max_s, max_i, min_h, min_s, min_i, type);
    Image.Msg_Subsribe();
	Image.Msg_Publisher();
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<SetParam>("set_param");
    char flag;
    SetParam srv;
    srv.request.name = PARAM_GAIN;
    srv.request.value = 10.0f;
    if(client.call(srv))
    {
      cout << "Yes" << endl;
    }
    else
    {
      cout << "No" << endl;
    }

    while(ros::ok())
    {
        Mat dst=Image.get_src();
        ros::spinOnce();
        waitKey(10);
    }
    cout << "Please" << endl;
    max_h=Image.get_max_h();
    min_h=Image.get_min_h();
    max_s=Image.get_max_s();
    min_s=Image.get_min_s();
    max_i=Image.get_max_i();
    min_i=Image.get_min_i();
    cout << max_h << endl;
    cout << "Please input the object you thresholded: " ;
    cout << " 'B' represents Ball, 'L' represents Line, 'F' represents Field, 'O' represents Obstacle " << endl;
    cin >> flag ;
    FILE *fp=fopen("threshold_value.txt","a+");
    fseek(fp,0l,SEEK_END);
    if(flag=='B')
    {
        fputs("\nBall_Threshold\n",fp);
    }
    else if(flag=='L')
    {
        fputs("\nLine_Threshold\n",fp);
    }
    else if(flag=='F')
    {
        fputs("\nField_Threshold\n",fp);
    }
    else if(flag=='O')
    {
        fputs("\nObstacle_Threshold\n",fp);
    }
    else
    {
        cout << "wrong type;Exiting" << endl;
    }
    cout << "recording threshold value from H->S->I by max->min" << endl;
    fprintf(fp,"%d %d %d %d %d %d\n",max_h,max_s,max_i,min_h,min_s,min_i); 
    fclose(fp);
}




















