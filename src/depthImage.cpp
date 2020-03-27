#include<opencv2/opencv.hpp>
#include<iostream>
 
using namespace cv;
using namespace std;
int main()
{
	Mat Img=imread("/home/limz/data/rgbd-data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", -1);
	if(!Img.data)  
    {  
        cout<<"could not open"<<endl;  
        return -1;  
    }  
	imshow("src",Img);
 
    cout<<Img.type()<<endl;
    
	for(int m = 0; m<Img.rows; m++) {
		for(int n = 0; n<Img.cols; n++) {
		//获取深度图中(m,n)处的值
		ushort d = Img.ptr<ushort>(m)[n];
		float z = double(d)/5000;
		cout<<z<<" ";
        }
        cout<<endl;
    }

	waitKey(0);
	return 0;
}
