#include "msg_transport.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RGB");
    RGB_Transport Image;
    for(;;)
    {
        Image.Msg_Subsribe();
        Mat dst=Image.get_src();
        if(dst.empty())
        {
            cout << "dst empty" << endl;
        }
        else
	{
  	    imshow("output_src",dst);
            waitKey(1);
	}
        ros::spin();
    }
}
