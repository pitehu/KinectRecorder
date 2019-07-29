#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include<iostream>
#include<time.h>
#include <stdio.h>
#include<sstream>
#include <direct.h>
#include <thread>
#include <ctime>
#include <mutex>
#include <ppl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;
using namespace cv;

std::mutex myMutex;
std::mutex myMutex2;
std::mutex myMutex3;
std::mutex myMutex4;

#ifndef SAFE_QUEUE
#define SAFE_QUEUE

#include <queue>
#include <mutex>
#include <condition_variable>

// A threadsafe-queue.
template <class T>
class SafeQueue
{
public:
	SafeQueue(void)
		: q()
		, m()
		, c()
	{}

	~SafeQueue(void)
	{}

	// Add an element to the queue.
	void push(T t)
	{
		std::lock_guard<std::mutex> lock(m);
		q.push(t);
		c.notify_one();
	}

	// Get the "front"-element.
	// If the queue is empty, wait till a element is avaiable.
	T pop(void)
	{
		std::unique_lock<std::mutex> lock(m);
		while (q.empty())
		{
			// release lock as long as the wait and reaquire it afterwards.
			c.wait(lock);
		}
		T val = q.front();
		q.pop();
		return val;
		
	}
	T front(void)
	{
		return q.front();
	}
	INT64 size(void)
	{
		return q.size();
	}
private:
	std::queue<T> q;
	mutable std::mutex m;
	std::condition_variable c;
};
#endif





//This function releases the pointers 

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}	
}
 

bool dirExists(const std::string& dirName_in)
{
	DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
	if (ftyp == INVALID_FILE_ATTRIBUTES)
		return false;  //something is wrong with your path!

	if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
		return true;   // this is a directory!

	return false;    // this is not a directory!
}

void mkdir(string saveaddr, string specificname)
{
	std::ostringstream name;
	name << saveaddr << specificname;
	if (!dirExists(name.str()))
	{
		const char * dirname = name.str().c_str();
		_mkdir(dirname);
	}
}


//This function loads the color data from the buffer and put it in a queue for furthure processing
void processcolor(INT64 &startime, IColorFrame* pColorFrame, SafeQueue<Mat> &colorqueue, SafeQueue<INT64> &colortimequeue)
{
	int width = 1920;
	int height = 1080;
	cv::Mat bufferMat(height, width, CV_8UC4);
	HRESULT hResult = S_OK;
	INT64 currentcolortime = 0;
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

	if (pColorFrame != nullptr) {
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
		if (SUCCEEDED(hResult))
		{
			hResult = pColorFrame->get_RelativeTime(&currentcolortime);
			if (startime == 0)  //Set the time of the first frame to 0
				startime = currentcolortime;

		}
		if (SUCCEEDED(hResult))
		{
			colorqueue.push(bufferMat);
			colortimequeue.push(currentcolortime - startime);
		}

	}
	bufferMat.release();
}

//This function loads the depth data(point cloud and depth frame mapped onto color frame) from the buffer and put it in a queue for furthure processing

void processdepth(ICoordinateMapper* pCoordinateMapper, INT64 &startime, IDepthFrame* depthframe, SafeQueue<pcl::PointCloud<pcl::PointXYZ>> &pcqueue, SafeQueue<Mat> &colorindepthqueue, SafeQueue<INT64> &pctimequeue, SafeQueue<INT64> &pctimequeue2)
{

	HRESULT hResult = S_OK;
	INT64 currentcolortime = 0;
	unsigned int sz;
	unsigned short* buf;
	cv::Mat depthincolor(1080, 1920, CV_16UC1);
	depthincolor = Scalar::all(0);
	CameraSpacePoint *depth2xyz = new CameraSpacePoint[512 * 424];
	ColorSpacePoint *depth2rgb = new ColorSpacePoint[512 * 424];
	cv::Mat depthMat(424, 512, CV_16UC1);

	typedef cv::Vec<float, 3> v3d;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	
	if (depthframe != nullptr)
	{
		hResult = depthframe->AccessUnderlyingBuffer(&sz, &buf);
		hResult = depthframe->CopyFrameDataToArray(512 * 424, reinterpret_cast<UINT16*>(depthMat.data));
		if (SUCCEEDED(hResult))
		{
			//Use the CoordinateMapper API to convert depth map into point cloud
			hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(424 * 512, buf, 424 * 512, depth2xyz); 
		
			if (SUCCEEDED(hResult))
			{
				depthframe->get_RelativeTime(&currentcolortime);
				if (startime == 0)
					startime = currentcolortime;
			}

		}

		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				INT64 k = i * j;
				if (isfinite(depth2xyz[k].X) and isfinite(depth2xyz[k].Y) and isfinite(depth2xyz[k].Z))
				{
					cloud.push_back(pcl::PointXYZ(depth2xyz[k].X, depth2xyz[k].Y, depth2xyz[k].Z));
				
				}
			}
		}


		hResult = pCoordinateMapper->MapDepthFrameToColorSpace(424 * 512, buf, 424 * 512, depth2rgb);
		//Convert the depth frame data into color space to use with color data together
		if (SUCCEEDED(hResult))
		{
			for (int i = 0; i < 424; i++)
				for (int j = 0; j < 512; j++)
				{
					int x = depth2rgb[i*j].X;
					int y = depth2rgb[i*j].Y;
					if (x > 0 and y > 0 and x < 1920 and y < 1080)
						depthincolor.at<UINT16>(y, x) = depthMat.at<UINT16>(i, j);

				}
			pcqueue.push(cloud);
			pctimequeue.push(currentcolortime - startime);
			pctimequeue2.push(currentcolortime - startime);
			colorindepthqueue.push(depthMat);

		}

	}
	delete depth2rgb;
	delete depth2xyz;
}

//This function loads the detected body pose information and put it in a queue for furthure processing
void processbody(ICoordinateMapper* pCoordinateMapper, INT64 &startime, IBodyFrame* pBodyFrame, string saveaddr)
{
	float Skeletons[6][25][3];
	HRESULT hResult = S_OK;
	INT64 currenttime;
	IBody* pBody[BODY_COUNT] = { 0 };
	if (pBodyFrame != nullptr)
	{


		hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);

		if (SUCCEEDED(hResult)) {
			pBodyFrame->get_RelativeTime(&currenttime);
			std::ostringstream folder;
			if (startime == 0)
				startime = currenttime;

		
			std::ostringstream name6;
			name6 << saveaddr << "images_skeletons/" << (currenttime - startime)/ 10000000. << ".txt";


			ofstream fout(name6.str());  

			for (int count = 0; count < BODY_COUNT; count++) {
				BOOLEAN bTracked = false;
				hResult = pBody[count]->get_IsTracked(&bTracked);
				if (SUCCEEDED(hResult) && bTracked) {
					Joint joint[JointType::JointType_Count];
					hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);//joint
					if (SUCCEEDED(hResult)) {


						CvPoint skeletonPoint[BODY_COUNT][JointType_Count] = { cvPoint(0,0) };
						// Joint
						for (int type = 0; type < JointType::JointType_Count; type++) {
							ColorSpacePoint colorSpacePoint = { 0 };
							pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
							int x = static_cast<int>(colorSpacePoint.X);
							int y = static_cast<int>(colorSpacePoint.Y);
							skeletonPoint[count][type].x = x;
							skeletonPoint[count][type].y = y;
						}

						fout << count << " ";
						for (int i = 0; i < 25; i++) {
							Skeletons[count][i][0] = joint[i].Position.X;
							Skeletons[count][i][1] = joint[i].Position.Y;
							Skeletons[count][i][2] = joint[i].Position.Z;
							//This saves the 2d pixel position of the joint on the RGB image as well as the corresponding depth
							fout << skeletonPoint[count][i].x << " " << skeletonPoint[count][i].y << " " << Skeletons[count][i][2] << " ";
						}
						fout << endl;
						for (int i = 0; i < 25; i++)
						{
							//This saves the 3d joint position of the detected human joint in camera space
							fout << joint[i].Position.X << " " << joint[i].Position.Y << " " << joint[i].Position.Z << " ";
						}
						fout << endl;

					}
				}

			}
			fout.close();
		}
	}

}
//fetch data from the queue and save it into .jpg image
	
void savecolor(SafeQueue<Mat> &colorqueue, SafeQueue<INT64> &colortimequeue, string saveaddr)
{

	while (1)
	{
		{
			cout << "colorqueue"<<colorqueue.size() << endl;
			std::ostringstream name4;
			name4 << saveaddr << "color/";
			name4 << colortimequeue.pop() / 10000000. << ".jpg";
			imwrite((name4.str()), colorqueue.pop());
		}
	}


}
//similar for depth and pc
void savedepth(SafeQueue<Mat> &colorindepthqueue, SafeQueue<INT64> &pctimequeue, string saveaddr)
{
	while (1)
	{
		{
			cout << "colorindepthqueue" << colorindepthqueue.size() << endl;
			std::ostringstream name5;
			name5 << saveaddr << "depthincolor/";

			name5 << pctimequeue.pop() / 10000000. << ".png";
			imwrite((name5.str()), colorindepthqueue.pop());

		}
	}
	
	
}


void savepc(SafeQueue<pcl::PointCloud<pcl::PointXYZ>> &pcqueue, SafeQueue<INT64> &pctimequeue, string saveaddr)
{
	while (1)
	{
		{
			cout << "pcqueue" << pcqueue.size() << endl;
			std::ostringstream name4;
			name4 << saveaddr << "pointcloud/";
			name4 << pctimequeue.pop() / 10000000. << ".pcd";
			pcl::io::savePCDFileBinary(name4.str(), pcqueue.pop());


		}
	}


}


int main(int argc, char *argv[])
{
	cv::setBreakOnError(true);
	setUseOptimized(true);
	
	// standard sensor initialization stuff
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor); 
	hResult = pSensor->Open();   
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}
	IMultiSourceFrameReader* reader;
	
	

	int width = 1920;
	int height = 1080;
	hResult = pSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth |
		FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body, &reader);
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);


	SafeQueue <Mat> colorqueue;
	SafeQueue <INT64>colortimequeue;
	SafeQueue <INT64>pctimequeue;
	SafeQueue <INT64>pctimequeue2;

	SafeQueue <pcl::PointCloud<pcl::PointXYZ>> pcqueue;
	SafeQueue <Mat> depthincolorqueue;

	cv::Mat bufferMat(height, width, CV_8UC4);
	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::Mat depthMat(424, 512, CV_16UC1);
	cv::Mat depthincolor(1080, 1920, CV_16UC1);

	cout << argv[0] << endl;
	cout << argv[1] << endl;
	
	string saveaddr(argv[1]);
	cout << saveaddr << endl;
	INT64 startime = 0;

	mkdir(saveaddr, "depthincolor/");
	mkdir(saveaddr, "color/");
	mkdir(saveaddr, "pointcloud/");
	mkdir(saveaddr, "images_skeletons/");

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);    // 创建map
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	//Use mutiple threads to save to ensure real time recording
	int piccount = 0;
	thread threads_depth[1] = {};
	for (int i = 0; i < 1; i++) {
		threads_depth[i] = std::thread(savedepth, std::ref(depthincolorqueue), std::ref(pctimequeue), std::ref(saveaddr));
		threads_depth[i].detach();
	}
	thread threads_pc[2] = {};
	for (int i = 0; i < 1 ; i++) {
		threads_pc[i] = std::thread(savepc, std::ref(pcqueue), std::ref(pctimequeue2), std::ref(saveaddr));
		threads_pc[i].detach();
	}
	thread threads_color[2] = {};
	for (int i = 0; i < 2; i++) {
		threads_color[i] = std::thread(savecolor, std::ref(colorqueue), std::ref(colortimequeue), std::ref(saveaddr));
		threads_color[i].detach();
	}


	while (1) {
		// Frame
		
		IColorFrame* pColorFrame = nullptr;
		IMultiSourceFrame* frame = nullptr;
		IDepthFrame* depthframe = nullptr;
		IBodyFrame* pBodyFrame = nullptr;

		hResult = reader->AcquireLatestFrame(&frame);
		if (SUCCEEDED(hResult))
		{

			IColorFrameReference* colorframeref = nullptr;
			hResult = frame->get_ColorFrameReference(&colorframeref);
			if (SUCCEEDED(hResult))
			{

				hResult = colorframeref->AcquireFrame(&pColorFrame);
			}
			SafeRelease(colorframeref);
		}


		
			if (frame!=nullptr)
			{
			IDepthFrameReference* depthframeref = nullptr;
			hResult=frame->get_DepthFrameReference(&depthframeref);
			if (SUCCEEDED(hResult))
			{
				hResult = depthframeref->AcquireFrame(&depthframe);
			}
			SafeRelease(depthframeref);
			}

			if (frame != nullptr)
			{
				IBodyFrameReference* bodyframeref = nullptr;
				hResult=frame->get_BodyFrameReference(&bodyframeref);
				if (SUCCEEDED(hResult))
				{
					hResult = bodyframeref->AcquireFrame(&pBodyFrame);
				}
				SafeRelease(bodyframeref);
			}
			processdepth(pCoordinateMapper, startime, depthframe, pcqueue, depthincolorqueue, pctimequeue,pctimequeue2);

			processcolor(startime,pColorFrame, colorqueue, colortimequeue);

			processbody(pCoordinateMapper, startime, pBodyFrame,saveaddr);

			SafeRelease(pColorFrame);
			SafeRelease(depthframe);
			SafeRelease(frame);
			SafeRelease(pBodyFrame);

		
	}


	SafeRelease(reader);
	SafeRelease(pCoordinateMapper);

	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);

	return 0;
}
