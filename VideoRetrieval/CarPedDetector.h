#ifndef CARPEDDETECTOR_H
#define CARPEDDETECTOR_H

#include "StrongTreeClassifier.h"
//人车检测器
#define MAXFEATURE 15000
#define VMELX 16
#define VMELY 30
struct CarPedObject
{
	int classify;                  //检测到的目标类别1 为车， 2位人，3为其他
	CvRect OutRect;                //检测到的目标在图像中的矩形框,包含了长宽大小 
};
struct SubHogfeature
{
	float hog36[36];   //一个block对应的hog特征 ,36维

};
class HogFeature
{
public:
	HogFeature();
	~HogFeature();
	void SetDim(int d);
	void ClearFeature(void);
	int Dim;      //HogFeature -〉一个搜索框对应一个hogfeature，Dim为对应的feature维度
	float*features;      //Dim个
	int x;
	int y;
	int width;
	int height;
	int mid_x;
	int mid_y;
	int nextfeature;    //下一个feature对应的索引值，考虑到meltbox，目的是为了形成链表结构，这里为了图方便就没有使用链表
    int lastfeature;    //上一个feature对应的索引值
    bool GoodOrNot;   //根据前景块判断对应的块是否有效
	int notype;    //用于融合时所属区域赋值
	static double DistanceBtwo(HogFeature* featureA, HogFeature* featureB);
};
class CCarPedDetector
{
public:
	CCarPedDetector(void);
	~CCarPedDetector(void);
	// 返回检测到的物体个数
	/*int FindCarOrPedObject(IplImage* DealImage, vector<CvSize2D32f>& SizeConversion, IplImage* foreImage, bool IsDCare, bool IsDPed,vector<CvRect>& detectedObject,CvRect frameImg);*/	
	void InitializeDetector(void);
	void CaculateGradient(IplImage* grayImage,CvMat*GradientX,CvMat*GradientY);
	// 判断y,x是否在block
	int InWhichCell(int y, int x,int CellSz);
	void ClearCarNo(void);
	/*void DrawAllTheCars(IplImage* showImage);*/
	/*void RealeaseData(void);*/
	// 将距离近的任意方框进行融合，返回最终得到的物体个数，以及新得到的hogfeature,
	/*int MeltBox(HogFeature** objectfeature, int numofobjects,double ValueForMelt);*/
	/*int MeltBox1(HogFeature** objectfeature, int numofobjects, double ValueForMelt);*/
	/*void DrawAllThePed(IplImage*showImage);*/
	// 判断给出灰度图片是人的概率，根据hog特征判断的话
	double IsImagePed(IplImage* WaitImg);
	int MeltBox(int numofobjects, HogFeature** objectfeature, double ValueForMelt_x, double ValueForMelt_y, IplImage* ForeBackImage);
	// 计算对应的框的前景比率
	double CaculateForeGRate(IplImage* ForeGrounde, CvRect& objectblob);
private:
	int iHeight;
	// 所有尺度下总共的特征数量
	int NumOfFeatures;
	// 检测到的汽车在AllHogfeature中的标识
	int CarNo[100];
	// 已经检测到的汽车数目
	int NumofCars;
	// 使用的尺度数目
	float scal[20];
	int num_scal;
	int iWeight;
	// 处理的图片宽度
	int iW;
	// 处理的图片高度
	int iH;
	// 一个block对应的特征维数
	int FeatDim;
	int NumofPeds;
	int CarWin_Block_x;
	int CarWin_Block_y;
	int PedWin_Block_x;
	int PedWin_Block_y;
	int PedBlocksInWin;
	int BlocksInWin;
	// 当前处理图像中的所有car_hog特征集合
	HogFeature* AllHogInImg[MAXFEATURE];
	// 当前处理图像中的所有ped_hog特征集合
	HogFeature* AllPedHogInImg[MAXFEATURE];
	// 用于车的分类器
	CStrongTreeClassifier* CarClassifier;
	//用于人的分类器
	CStrongTreeClassifier* PedClassifier;
	vector<CarPedObject> m_NewObjectList; 
	int	Ped_M;     //人的搜索框尺寸 M*N
	int Ped_N;
	int Car_M;     //车的搜索框尺寸 M*N
	int Car_N;
	int SkipStep; //窗口步长
	//int BlockSkipStep;   //Block步长
	int CellSz;   //cell对应的尺度 cellsz*cellsz
	int BlockSz;  //我们采用一个Block中包含4个Cell的结构，所以BlockSz=CellSz*2
	int BinNum;   //方向数 ，我们采用9个
	int Angle;    //180度，不考虑360度，因为背景跟前景的差度不明确

	vector<HogFeature> PedHogInImg;
public:
	int StatisticTheXPixel(IplImage* ForeGround,IplImage*orignImage,int minAscend,int possiblePwidth);
	double FindPedObject(IplImage* DealImage, IplImage* foreImage,vector<CvRect>& detectedObject,CvRect frameImg,CvSize WFrameSize,double SizeAlpha);
	int StatisticTheXPixelandH(IplImage* ForeGround, IplImage* orignImage, int minAscend, int possiblePwidth,vector<int>& AcmeX);
	double SizeAlpha;
private:
	int baseN;
	int SizeTrainN;
};

#endif