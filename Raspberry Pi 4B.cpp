#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>


using namespace std;
using namespace cv;
using namespace raspicam;

Mat frame, Matrix, framePers, frameGray, framethresh, frameEdge, frameFinal, ROILane, ROILaneEnd, frameFinalDuplicate, frameFinalDuplicate1;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd;


RaspiCam_Cv Camera;

stringstream ss;

vector<int> histogramLane;
vector<int> histogramLaneEnd;

Point2f Source[] = {Point2f(80,185), Point2f(325,185), Point2f(55,225), Point2f(345,225)};
Point2f Destination[] = {Point2f(60,0), Point2f(300,0), Point2f(60,240), Point2f(300,240)};


 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,100));

}

void Perspective()
{
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
	
	Matrix = getPerspectiveTransform(Source, Destination);
	warpPerspective(frame, framePers, Matrix, Size(400,240));
	
}


void Treshold()
{
	cvtColor (framePers, frameGray, COLOR_RGB2GRAY);
	inRange(frameGray, 43, 320, framethresh);
	Canny(frameGray, frameEdge, 60, 255, 3, false);
	add(framethresh, frameEdge, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);
	cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);
}

void Histogram()
{
	histogramLane.resize(400);
	histogramLane.clear();
	
	for(int i=0;i<400; i++)
	{
		ROILane = frameFinalDuplicate(Rect(i,140,1,100));
		divide(255, ROILane, ROILane);
		histogramLane.push_back((int)(sum(ROILane)[0]));
	}
	
	histogramLaneEnd.resize(400);
	histogramLaneEnd.clear();
	
	for(int i=0;i<400; i++)
	{
		ROILaneEnd = frameFinalDuplicate1(Rect(i,0,1,240));
		divide(255, ROILaneEnd, ROILaneEnd);
		histogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));
	}
	
	laneEnd = sum(histogramLaneEnd)[0];
	cout<<"Lane End = "<<laneEnd;
}

void LaneFinder()
{
	vector<int>:: iterator LeftPtr;
	LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() +150);
	LeftLanePos = distance (histogramLane.begin(), LeftPtr);
	
	vector<int>:: iterator RightPtr;
	RightPtr = max_element(histogramLane.begin() +250, histogramLane.end());
	RightLanePos = distance (histogramLane.begin(), RightPtr);
	
	line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0,255,0), 2);
	line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
}
void LaneCenter()
{
	laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
	frameCenter = 178;
	line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter, 240), Scalar(0,255,0), 3);
	line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter, 240), Scalar(255,0,0), 3);
	
	Result = laneCenter-frameCenter;
	
}

void Capture()
{
	Camera.grab();
    Camera.retrieve( frame);
    cvtColor(frame, frame, COLOR_BGR2RGB);
}

int main(int argc,char **argv)
{
	wiringPiSetup();
	pinMode(21, OUTPUT);
	pinMode(22, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(24, OUTPUT);
	
	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())
	{
		
	cout<<"Failed to Connect"<<endl;
     }
     
     cout<<"Camera Id = "<<Camera.getId()<<endl;
     
     
     
    
    while(1)
    {
    auto start = std::chrono::system_clock::now();
    Capture();
    Perspective();
    Treshold();
    Histogram();
    LaneFinder();
    LaneCenter();
    
    if(laneEnd > 7500)
    {
		digitalWrite(21,1);
		digitalWrite(22,1);             //decimal =7
		digitalWrite(23,1);
		digitalWrite(24,0);
		cout<<"LaneEnd"<<endl;
	}
    
    if (Result==0)
    {
		digitalWrite(21,0);
		digitalWrite(22,0);             //decimal =0
		digitalWrite(23,0);
		digitalWrite(24,0);
		cout<<"Forward"<<endl;
	}
	
	  
    else if (Result >0 && Result <10)
    {
		digitalWrite(21,1);
		digitalWrite(22,0);             //decimal =1
		digitalWrite(23,0);
		digitalWrite(24,0);
		cout<<"Right1"<<endl;
	}
    
    	  
    else if (Result >=10 && Result <20)
    {
		digitalWrite(21,0);
		digitalWrite(22,1);             //decimal =2
		digitalWrite(23,0);
		digitalWrite(24,0);
		cout<<"Right2"<<endl;
	}
    
     else if (Result >20)
    {
		digitalWrite(21,1);
		digitalWrite(22,1);             //decimal =3
		digitalWrite(23,0);
		digitalWrite(24,0);
		cout<<"Right3"<<endl;
	}
	
	  
    else if (Result <0 && Result >-10)
    {
		digitalWrite(21,0);
		digitalWrite(22,0);             //decimal =4
		digitalWrite(23,1);
		digitalWrite(24,0);
		cout<<"Left1"<<endl;
	}
    
    	  
    else if (Result <=-10 && Result >-20)
    {
		digitalWrite(21,1);
		digitalWrite(22,0);             //decimal =5
		digitalWrite(23,1);
		digitalWrite(24,0);
		cout<<"Left2"<<endl;
	}
    
     else if (Result <-20)
    {
		digitalWrite(21,0);
		digitalWrite(22,1);             //decimal =6
		digitalWrite(23,1);
		digitalWrite(24,0);
		cout<<"Left3"<<endl;
	}
	
	if (laneEnd > 7000)
{
    ss.str(" ");
    ss.clear();
    ss<<"Lane End";
    putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,0), 2);
}
    
    
    else if (Result ==0)
{
    ss.str(" ");
    ss.clear();
    ss<<"Result = "<<Result<<" (Move Forward)";
    putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
}

    else if (Result > 0)
    {
    ss.str(" ");
    ss.clear();
    ss<<"Result = "<<Result<<" (Move Right)";
    putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
}



    else if (Result < 0)
    {
    ss.str(" ");
    ss.clear();
    ss<<"Result = "<<Result<<" (Move Left)";
    putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
}

    namedWindow("original", WINDOW_KEEPRATIO);
    moveWindow("original", 50, 100);
    resizeWindow("original", 640, 480);
    imshow("original", frame);
    
    namedWindow("Perspective", WINDOW_KEEPRATIO);
    moveWindow("Perspective", 700, 100);
    resizeWindow("Perspective", 640, 480);
    imshow("Perspective", framePers);
    
    namedWindow("Final", WINDOW_KEEPRATIO);
    moveWindow("Final", 1340, 100);
    resizeWindow("Final", 640, 480);
    imshow("Final", frameFinal);
    
    
    waitKey(1);
    auto end = std::chrono::system_clock::now();
  

    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
  
    
    
    
    }

    
    return 0;
     
}
