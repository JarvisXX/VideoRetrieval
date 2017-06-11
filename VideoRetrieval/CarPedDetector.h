#ifndef CARPEDDETECTOR_H
#define CARPEDDETECTOR_H

#include "StrongTreeClassifier.h"
//�˳������
#define MAXFEATURE 15000
#define VMELX 16
#define VMELY 30
struct CarPedObject
{
	int classify;                  //��⵽��Ŀ�����1 Ϊ���� 2λ�ˣ�3Ϊ����
	CvRect OutRect;                //��⵽��Ŀ����ͼ���еľ��ο�,�����˳����С 
};
struct SubHogfeature
{
	float hog36[36];   //һ��block��Ӧ��hog���� ,36ά

};
class HogFeature
{
public:
	HogFeature();
	~HogFeature();
	void SetDim(int d);
	void ClearFeature(void);
	int Dim;      //HogFeature -��һ���������Ӧһ��hogfeature��DimΪ��Ӧ��featureά��
	float*features;      //Dim��
	int x;
	int y;
	int width;
	int height;
	int mid_x;
	int mid_y;
	int nextfeature;    //��һ��feature��Ӧ������ֵ�����ǵ�meltbox��Ŀ����Ϊ���γ�����ṹ������Ϊ��ͼ�����û��ʹ������
    int lastfeature;    //��һ��feature��Ӧ������ֵ
    bool GoodOrNot;   //����ǰ�����ж϶�Ӧ�Ŀ��Ƿ���Ч
	int notype;    //�����ں�ʱ��������ֵ
	static double DistanceBtwo(HogFeature* featureA, HogFeature* featureB);
};
class CCarPedDetector
{
public:
	CCarPedDetector(void);
	~CCarPedDetector(void);
	// ���ؼ�⵽���������
	/*int FindCarOrPedObject(IplImage* DealImage, vector<CvSize2D32f>& SizeConversion, IplImage* foreImage, bool IsDCare, bool IsDPed,vector<CvRect>& detectedObject,CvRect frameImg);*/	
	void InitializeDetector(void);
	void CaculateGradient(IplImage* grayImage,CvMat*GradientX,CvMat*GradientY);
	// �ж�y,x�Ƿ���block
	int InWhichCell(int y, int x,int CellSz);
	void ClearCarNo(void);
	/*void DrawAllTheCars(IplImage* showImage);*/
	/*void RealeaseData(void);*/
	// ������������ⷽ������ںϣ��������յõ�������������Լ��µõ���hogfeature,
	/*int MeltBox(HogFeature** objectfeature, int numofobjects,double ValueForMelt);*/
	/*int MeltBox1(HogFeature** objectfeature, int numofobjects, double ValueForMelt);*/
	/*void DrawAllThePed(IplImage*showImage);*/
	// �жϸ����Ҷ�ͼƬ���˵ĸ��ʣ�����hog�����жϵĻ�
	double IsImagePed(IplImage* WaitImg);
	int MeltBox(int numofobjects, HogFeature** objectfeature, double ValueForMelt_x, double ValueForMelt_y, IplImage* ForeBackImage);
	// �����Ӧ�Ŀ��ǰ������
	double CaculateForeGRate(IplImage* ForeGrounde, CvRect& objectblob);
private:
	int iHeight;
	// ���г߶����ܹ�����������
	int NumOfFeatures;
	// ��⵽��������AllHogfeature�еı�ʶ
	int CarNo[100];
	// �Ѿ���⵽��������Ŀ
	int NumofCars;
	// ʹ�õĳ߶���Ŀ
	float scal[20];
	int num_scal;
	int iWeight;
	// �����ͼƬ���
	int iW;
	// �����ͼƬ�߶�
	int iH;
	// һ��block��Ӧ������ά��
	int FeatDim;
	int NumofPeds;
	int CarWin_Block_x;
	int CarWin_Block_y;
	int PedWin_Block_x;
	int PedWin_Block_y;
	int PedBlocksInWin;
	int BlocksInWin;
	// ��ǰ����ͼ���е�����car_hog��������
	HogFeature* AllHogInImg[MAXFEATURE];
	// ��ǰ����ͼ���е�����ped_hog��������
	HogFeature* AllPedHogInImg[MAXFEATURE];
	// ���ڳ��ķ�����
	CStrongTreeClassifier* CarClassifier;
	//�����˵ķ�����
	CStrongTreeClassifier* PedClassifier;
	vector<CarPedObject> m_NewObjectList; 
	int	Ped_M;     //�˵�������ߴ� M*N
	int Ped_N;
	int Car_M;     //����������ߴ� M*N
	int Car_N;
	int SkipStep; //���ڲ���
	//int BlockSkipStep;   //Block����
	int CellSz;   //cell��Ӧ�ĳ߶� cellsz*cellsz
	int BlockSz;  //���ǲ���һ��Block�а���4��Cell�Ľṹ������BlockSz=CellSz*2
	int BinNum;   //������ �����ǲ���9��
	int Angle;    //180�ȣ�������360�ȣ���Ϊ������ǰ���Ĳ�Ȳ���ȷ

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