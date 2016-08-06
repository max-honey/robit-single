#include "msg_transport.h"

using namespace gige;
using namespace gige_cap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RGB");

    int max_h=0, max_s=0, max_i=0;
    int min_h=0, min_s=0, min_i=0;
    char type;
    char s[100];
    FILE *fp;
    if((fp= fopen("/home/robit/catkin_ws/threshold_value.txt", "r"))==NULL)
    {
	    ROS_ERROR("file does not excist");
        return 0;
    }
    else
    {
	cout << "Please input the object you thresholded: " ;
        cout << " 'B' represents Ball, 'L' represents Line, 'F' represents Field, 'O' represents Obstacle " << endl;
	//cin >> type ;
  type = 'B';
        for(int i=0;i<12;i++)
    	{
	    fgets(s, 100, fp);
            if(s[0]==type)
	    {
		fscanf(fp, "%d %d %d %d %d %d", &max_h,&max_s,&max_i,&min_h,&min_s,&min_i);
	    }
	}
        //cout << max_h << " " << max_s << " " << max_i << endl;
    }

    RGB_Transport Image;
    Image.Msg_Subsribe();
    Image.Msg_Publisher();
    Image.set_value(max_h, max_s, max_i, min_h, min_s, min_i, type);
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<SetParam>("set_param");
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
}
