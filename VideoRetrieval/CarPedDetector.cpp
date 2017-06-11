#include "clude.h"
#include "CarPedDetector.h"
#define PedHalfSearchWidth 16

CCarPedDetector::CCarPedDetector(void)
: iW(0)
, iH(0)
, FeatDim(0)
, num_scal(0)
, iWeight(0)
, iHeight(0)
, NumOfFeatures(0)
, CarClassifier(NULL)
, NumofCars(0)
,PedClassifier(NULL)

, NumofPeds(0)
, CarWin_Block_x(0)
, CarWin_Block_y(0)
, PedWin_Block_x(0)
, PedWin_Block_y(0)
, PedBlocksInWin(0)
, BlocksInWin(0)
, SizeAlpha(0.25)
, baseN(100)
, SizeTrainN(0)
{
	scal[0]=1;
    scal[1]=0.88;
	scal[2]=0.72;
	scal[3]=0.60;
	scal[4]=0.56;
	scal[5]=0.50;
	scal[6]=0.45;
	scal[7]=0.40;
	scal[8]=0.38;
	scal[9]=0.36;
	//   vsc=[  1      0.707106781186548      
	//0.594603557501360       0.500000000000000     0.420448207626857]; 
// 	scal[0]=0.5;
// 	scal[1]=0.7071067;
// 	scal[2]=0.594603557;
// 	scal[3]=0.56;
// 	scal[4]=0.62;
// 	scal[5]=0.76;
// 	scal[6]=0.73;
// 	scal[7]=0.96;
// 	scal[8]=0.55;
// 	scal[9]=0.5;


}

CCarPedDetector::~CCarPedDetector(void)
{

	if(NumofCars!=0)
	{
		for(int i=0;i<NumofCars;i++)
		{
			delete AllHogInImg[i];
		}
	}
	if(NumofPeds!=0)
	{
		for(int i=0;i<NumofPeds;i++)
		{
			delete AllPedHogInImg[i];
		}
	}
	PedHogInImg.clear();
	if(CarClassifier!=NULL)
		delete CarClassifier;
	if(PedClassifier!=NULL)
		delete PedClassifier;
}

HogFeature::HogFeature(void)
{
    Dim=0;
	x=0;
	y=0;
	width=0;
	height=0;
	nextfeature=-2;   //��һ��������ʼ��Ϊ-1
	lastfeature=-2;
	features=NULL;
	notype=0;
	
}
HogFeature::~HogFeature(void)
{
	if(features!=NULL)
		delete features;
}
void HogFeature::SetDim(int d)   //�趨features
{

	Dim=d;
	features=new float[Dim];

}
void HogFeature::ClearFeature(void)
{
	Dim=0;
	if(features!=NULL)
	delete features;
	features=NULL;
}
void CCarPedDetector::InitializeDetector(void)
{
	Ped_M=128;     //�˵�������ߴ� M*N
	Ped_N=64;
	Car_M=64;      //����������ߴ� M*N
	Car_N=128;
	SkipStep=8;    //blockÿһ���ƶ�����  ,ʵ�����ظ����� ����Ϊ��img�в�����block��Ȼ���ظ���
	//BlockSkipStep=8;
	CellSz=8;      //cell��Ӧ�ĳ߶� cellsz*cellsz
	BlockSz=16;   //���ǲ���һ��Block�а���4��Cell�Ľṹ������BlockSz=CellSz*2
	BinNum=9;     //������ �����ǲ���9��
	Angle=180;    //180�ȣ�������360�ȣ���Ϊ������ǰ���Ĳ�Ȳ���ȷ
	FeatDim=BinNum*4;
	//��ʼ������������
	CarClassifier=new CStrongTreeClassifier;
	CarClassifier->InitialStrongClassifier(NUMOFWEAK,"weakteset.txt");
	CarClassifier->classvalue=6.5;      //?������ֵ
    //��ʼ���˵ķ�����
	PedClassifier=new CStrongTreeClassifier;
	PedClassifier->InitialStrongClassifier(PEDNUMOFWEAK,"pedstrongclass.txt");
	PedClassifier->classvalue=10;        //���˷�����ֵ
 //   CarClassifier->TestReadWrite("K:\\weakteset1.txt");
	CarWin_Block_x=floor(double(Car_N-BlockSz)/SkipStep+1);
	CarWin_Block_y=floor(double(Car_M-BlockSz)/SkipStep+1);
	PedWin_Block_x=floor(double(Ped_N-BlockSz)/SkipStep+1);
	PedWin_Block_y=floor(double(Ped_M-BlockSz)/SkipStep+1);
	PedBlocksInWin=PedWin_Block_x*PedWin_Block_y;

	BlocksInWin=CarWin_Block_x*CarWin_Block_y;  //һ��������win����������blcok��Ŀ
	for(int i=0;i<100;i++)
	{
		CarNo[i]=-1;
	}


}
void CCarPedDetector::CaculateGradient(IplImage* grayImage,CvMat*GradientX,CvMat*GradientY)
{
	CvMat*GRAY1,headgray;
	
	CvMat*GRAY=cvCreateMat(iH,iW,CV_32FC1);
	//cvInitMatHeader( GRAY, iH, iW, CV_32FC1 );//��ʼ��

	GRAY1=cvGetMat(grayImage,&headgray);
    cvConvertScale( GRAY1, GRAY);
	ofstream outfile2;
	outfile2.open("K:\\GRAY.txt",ios::trunc);
	for(int i=0;i<iH;i++)
	{
		for(int j=0;j<20;j++)
		{
			outfile2<<cvmGet(GRAY,i,j)<<" ";
		}
		outfile2<<"\n";
	}
	outfile2.close();
	 //����GradientX
	for(int i=0;i<iH;i++)
	{
		double tvalue=0;
		tvalue=(cvmGet(GRAY,i,1)-cvmGet(GRAY,i,0));
		if(tvalue==0)
		{
			tvalue=0.00001;
		}
         cvmSet(GradientX,i,0,tvalue);
	    tvalue=(cvmGet(GRAY,i,iW-1)-cvmGet(GRAY,i,iW-2));
		if(tvalue==0)
		{
			tvalue=0.00001;
		}
        cvmSet(GradientX,i,iW-1,tvalue);
	}
	for(int j=1;j<iW-1;j++)
		for(int i=0;i<iH;i++)
		{
			double tvalue=0;
			tvalue=(cvmGet(GRAY,i,j+1)-cvmGet(GRAY,i,j-1));
			if(tvalue==0)
			{
				tvalue=0.00001;
			}			
			cvmSet(GradientX,i,j,tvalue);
		}
//����GradientY
		for(int i=0;i<iW;i++)
	{
         cvmSet(GradientY,0,i,(cvmGet(GRAY,1,i)-cvmGet(GRAY,0,i)));
         cvmSet(GradientY,iH-1,i,(cvmGet(GRAY,iH-1,i)-cvmGet(GRAY,iH-2,i)));
	}

	for(int i=1;i<iH-1;i++)
	  for(int j=0;j<iW;j++)
		{
			cvmSet(GradientY,i,j,(cvmGet(GRAY,i+1,j)-cvmGet(GRAY,i-1,j)));
		}
	cvReleaseMat(&GRAY);

}

// �ж�y,x������һ��cell
int CCarPedDetector::InWhichCell(int y, int x,int CellSz)
{
   int typearea=0;
   if(y<CellSz&&y>=0&&x<CellSz&&x>=0)
	   typearea=1;
   if(x>=CellSz&&x<2*CellSz&&y<CellSz&&y>=0)
       typearea=2;
   if(x<CellSz&&x>=0&&y>=CellSz&&y<2*CellSz)
	   typearea=3;
   if(x>=CellSz&&x<2*CellSz&&y>=CellSz&&y<2*CellSz)
	   typearea=4;

	return typearea;
}

void CCarPedDetector::ClearCarNo(void)
{
	for(int i=0;i<NumofCars;i++)
	{
		CarNo[i]=-1;
	}
}

//void CCarPedDetector::DrawAllTheCars(IplImage* showImage)
//{
//	for(int i=0;i<NumofCars;i++)
//	{
//		if(AllHogInImg[i]!=NULL)
//		{
//         cvRectangle(showImage,cvPoint(AllHogInImg[i]->x,AllHogInImg[i]->y),
//			 cvPoint(AllHogInImg[i]->x+AllHogInImg[i]->width,AllHogInImg[i]->y+AllHogInImg[i]->height),CV_RGB(255,0,0),2);
//		}
//	}
//}

//void CCarPedDetector::RealeaseData(void)
//{
//	if(NumofCars!=0)
//	{
//		for(int i=0;i<NumofCars;i++)
//		{
//             delete AllHogInImg[i];
//		}
//		NumofCars=0;
//	}
//	if(NumofPeds!=0)
//	{
//		for(int i=0;i<NumofPeds;i++)
//		{
//			delete AllPedHogInImg[i];
//		}
//		NumofPeds=0;
//	}
//	PedHogInImg.clear();
//}

// ������������ⷽ������ںϣ��������յõ�������������Լ��µõ���hogfeature,
//int CCarPedDetector::MeltBox(HogFeature** objectfeature, int numofobjects,double ValueForMelt)
//{
//
//	int newNumObjects=numofobjects;
//	double mindistance=ValueForMelt-1;
//
//	mindistance=ValueForMelt-1;
//	while(mindistance<ValueForMelt)
//	{
//		mindistance=ValueForMelt;
//		int min_i=-1;  //��¼������̵���������
//		int min_j=-1;
//        for(int i=0;i<newNumObjects-1;i++)
//			for(int j=i+1;j<newNumObjects;j++)
//			{
// 			   double DistanceBtTO=0;
// 			   DistanceBtTO=HogFeature::DistanceBtwo(objectfeature[i],objectfeature[j]);  
//			   if(DistanceBtTO<mindistance)
//			   {
//				   min_i=i;
//				   min_j=j;
//				   mindistance=DistanceBtTO;
//			   }
//
//			}
//			if(min_i!=-1)
//			{
//
//			    	objectfeature[min_i]->mid_x=(objectfeature[min_i]->mid_x+objectfeature[min_j]->mid_x)/2.0;
//			    	objectfeature[min_i]->mid_y=(objectfeature[min_i]->mid_y+objectfeature[min_j]->mid_y)/2.0;
//					objectfeature[min_i]->width=(objectfeature[min_i]->width+objectfeature[min_j]->width)/2.0;
//					objectfeature[min_i]->height=(objectfeature[min_i]->height+objectfeature[min_j]->height)/2.0;
//					objectfeature[min_i]->x=objectfeature[min_i]->mid_x-objectfeature[min_i]->width/2.0;
//		    		objectfeature[min_i]->y=objectfeature[min_i]->mid_y-objectfeature[min_i]->height/2.0;
//	     			objectfeature[min_j]->ClearFeature();
//					delete objectfeature[min_j];		
//			        for(int i=0;i<newNumObjects;i++)
//			        {
//				      if(i>min_j)
//				       {
//					     objectfeature[i-1]=objectfeature[i];
//				        }
//			         }
//					objectfeature[newNumObjects-1]=NULL;
//					newNumObjects--;
//			}
//	}
//
//	return newNumObjects;
//}

double HogFeature::DistanceBtwo(HogFeature* featureA, HogFeature* featureB)
{
    double dist=(featureA->mid_x-featureB->mid_x)*(featureA->mid_x-featureB->mid_x)
		+(featureA->mid_y-featureB->mid_y)*(featureA->mid_y-featureB->mid_y);
	dist=sqrt(dist);
	return dist;
}
//����⵽���ˣ��������ں�
//int CCarPedDetector::MeltBox1(HogFeature** objectfeature, int numofobjects, double ValueForMelt)
//{
//	int newNumObjects=numofobjects;
//	double mindistance=ValueForMelt-1;
//
//	mindistance=ValueForMelt-1;
//	while(mindistance<ValueForMelt)
//	{
//		mindistance=ValueForMelt;
//		for(int i=0;i<newNumObjects-1;i++)
//			for(int j=i+1;j<newNumObjects;j++)
//			{
//				if((objectfeature[i]!=NULL)&&(objectfeature[j]!=NULL))
//				{
//					double DistanceBtTO=0;
//					DistanceBtTO=HogFeature::DistanceBtwo(objectfeature[i],objectfeature[j]);  
//					if(DistanceBtTO<ValueForMelt)
//					{
//						mindistance=ValueForMelt;
//						objectfeature[i]->mid_x=(objectfeature[i]->mid_x+objectfeature[j]->mid_x)/2.0;
//						objectfeature[i]->mid_y=(objectfeature[i]->mid_y+objectfeature[j]->mid_y)/2.0;
//						objectfeature[i]->width=(objectfeature[i]->width+objectfeature[j]->width)/2.0;
//						objectfeature[i]->height=(objectfeature[i]->height+objectfeature[j]->height)/2.0;
//						objectfeature[i]->x=objectfeature[i]->mid_x-objectfeature[i]->width/2.0;
//						objectfeature[i]->y=objectfeature[i]->mid_y-objectfeature[i]->height/2.0;
//						objectfeature[j]->ClearFeature();
//						delete objectfeature[j];
//						objectfeature[j]=NULL;
//					}
//				}
//			}
//		int record=0;
//		for(int i=0;i<newNumObjects;i++)
//		{
//			if(objectfeature[i]==NULL)
//			{
//                  record++;
//				  continue;
//			}
//			objectfeature[i-record]=objectfeature[i];
//		}
//		newNumObjects-=record;
//	}
//	return newNumObjects;
//}

//void CCarPedDetector::DrawAllThePed(IplImage*showImage)
//{
//	for(int i=0;i<NumofPeds;i++)
//	{
//		if(AllPedHogInImg[i]!=NULL)
//		{
//			cvRectangle(showImage,cvPoint(AllPedHogInImg[i]->x,AllPedHogInImg[i]->y),
//				cvPoint(AllPedHogInImg[i]->x+AllPedHogInImg[i]->width,AllPedHogInImg[i]->y+AllPedHogInImg[i]->height),CV_RGB(0,255,0),2);
//		}
//	}
//}

// �жϸ����Ҷ�ͼƬ���˵ĸ��ʣ�����hog�����жϵĻ�
double CCarPedDetector::IsImagePed(IplImage* WaitImg)
{
	iW=WaitImg->width;
	iH=WaitImg->height;
	int iWeight=iW;
	int iHeight=iH;
	IplImage*FgrayImg;
	double possible_result=0;
	FgrayImg=cvCreateImage(cvSize(iW,iH),IPL_DEPTH_8U,1);

	//���������ͼ���ǻҶ�ͼ����ת���ɻҶ�ͼ����
	if(WaitImg->nChannels==3)
	{
		cvCvtColor(WaitImg, FgrayImg, CV_RGB2GRAY);	
	}
	else
	{
		cvCopy(WaitImg,FgrayImg);
	}
	HogFeature*CurrentHog;

	IplImage*grayImg;
	grayImg=cvCreateImage(cvSize(Ped_N,Ped_M),IPL_DEPTH_8U,1);
	cvResize(FgrayImg,grayImg,CV_INTER_LINEAR);
    iW=Ped_N;
	iH=Ped_M;
	int Ped_nwin_x=1;    //��ͼ��DealImage����Ҫ������������� nwin_x*nwin_y 
	int Ped_nwin_y=1;

	int PedWin=1;
	//������м������һ��hogmap����ӳimg��Ӧ��hog����ͼ
	//ͼ��Img��Ӧ���ܵ�Block��Ŀ  Bnwin_x*Bnwin_y
	int Bnwin_x=floor(double(Ped_N-2*CellSz)/SkipStep+1);
	int Bnwin_y=floor(double(Ped_M-2*CellSz)/SkipStep+1);
	int SumOfBlocks=Bnwin_y*Bnwin_x;    //һ��ͼ�����е�block��������blocksize��block�ĳ��̶ܳȾ���
	SubHogfeature*hogmap=new SubHogfeature[SumOfBlocks];   //����ͼ��
	//����Gradient
	CvMat*GradientX=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	CvMat*GradientY=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	CaculateGradient(grayImg,GradientX,GradientY);
	// speedup
	//cvNamedWindow("s");
	//cvShowImage("s", grayImg);
	//����������magnitudeֱ��ͼ
	CvMat*GMag=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	//���㷽��ͼXY
	CvMat*YX=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	int nAngle=20;
	for(int i=0;i<Ped_M;i++)
	{
		for(int j=0;j<Ped_N;j++)
		{
			double tvalue=0;
			double tvalue1=0;
			tvalue=sqrt(cvmGet(GradientX,i,j)*cvmGet(GradientX,i,j)+cvmGet(GradientY,i,j)*cvmGet(GradientY,i,j));
			if(tvalue==0)
			{
				tvalue=0.00001;
			}
			cvmSet(GMag,i,j,tvalue);
			tvalue=double(cvmGet(GradientY,i,j))/cvmGet(GradientX,i,j);
			tvalue1=(atan(tvalue)+3.1415926/2)*180/3.1415926;	//ת��Ϊ����	  
			int temptdirect;
			temptdirect=ceil(tvalue1/double(nAngle));
			cvmSet(YX,i,j,temptdirect);
		}
	}
	///////////////////////////////////����haar����ͼSubHogfeature*hogmap
	for(int i=0;i<Bnwin_y;i++)
	{
		for(int j=0;j<Bnwin_x;j++)
		{
			int offsetx=j*SkipStep;
			int offsety=i*SkipStep;
			SubHogfeature t_feature;
			for(int k=0;k<36;k++)
				t_feature.hog36[k]=0;
			double sumf=0;    //��һ�����ӣ���ÿһblock��Ҫ���й�һ��
			for(int y=0;y<CellSz*2;y++)
			{
				for(int x=0;x<CellSz*2;x++)
				{
					int tempt=InWhichCell(y,x,CellSz);
					int index=cvmGet(YX,y+offsety,x+offsetx);
					double mag=cvmGet(GMag,y+offsety,x+offsetx);
					t_feature.hog36[(tempt-1)*BinNum+index-1]+=mag;
				}
			}

			for(int k=0;k<36;k++)
			{
				sumf+=t_feature.hog36[k]*t_feature.hog36[k];
			}
			double factor=sqrt(sumf);
			for(int k=0;k<36;k++)
			{
				t_feature.hog36[k]=t_feature.hog36[k]/(factor+0.000001);
			}
			int tposition=0;
			tposition=Bnwin_x*i+j;
			hogmap[tposition]=t_feature;
		}
	}
	////������д��ڵ�����
	//�������д����ҳ���
	//�������д����ҳ�����
 	float*tempt_feature;
 	tempt_feature=new float[BlocksInWin*36];
// 
// 		for(int i=0;i<Ped_nwin_x;i++)
// 			for(int j=0;j<Ped_nwin_y;j++)
// 			{
 				int count_blocks=0;
 				for(int x=0;x<PedWin_Block_x;x++)
 					for(int y=0;y<PedWin_Block_y;y++)
 					{
 						int tposition=0;  
 						tposition=Bnwin_x*(y)+x;
 						for(int k=0;k<36;k++)
						{
							int tempt=count_blocks*36+k;
							tempt_feature[tempt]=hogmap[tposition].hog36[k];
						}
						count_blocks++;
					}
// 					int tempt1=0;
 					possible_result=PedClassifier->Classify(tempt_feature);
// 					if(tempt1==1)
// 					{
// 						AllPedHogInImg[NumofPeds]=new HogFeature;	
// 						AllPedHogInImg[NumofPeds]->x=float(offsetx)/scal[svc];
// 						AllPedHogInImg[NumofPeds]->y=float(offsety)/scal[svc];
// 						AllPedHogInImg[NumofPeds]->width=Ped_N/scal[svc];
// 						AllPedHogInImg[NumofPeds]->height=Ped_M/scal[svc];
// 						AllPedHogInImg[NumofPeds]->mid_x=AllPedHogInImg[NumofPeds]->x+AllPedHogInImg[NumofPeds]->width/2.0;
// 						AllPedHogInImg[NumofPeds]->mid_y=AllPedHogInImg[NumofPeds]->y+AllPedHogInImg[NumofPeds]->height/2.0;
// 						if(NumofPeds!=0)
// 						{
// 							AllPedHogInImg[NumofPeds-1]->nextfeature=NumofPeds;
// 							AllPedHogInImg[NumofPeds]->lastfeature=NumofPeds-1;
// 						}
// 						else
// 						{
// 							AllPedHogInImg[NumofPeds]->lastfeature=-1;
// 						}
// 						NumofPeds++;	
// 					}
// 			}

			//�ͷ���ʱʹ�õ���Դ
			cvReleaseImage(&grayImg);
			cvReleaseMat(&GradientX);
			cvReleaseMat(&GradientY);
			cvReleaseMat(&YX);
			cvReleaseMat(&GMag);
			delete tempt_feature;
			delete hogmap;

			
	return possible_result;
}

//int CCarPedDetector::FindCarOrPedObject(IplImage* DealImage, vector<CvSize2D32f>& SizeConversion, IplImage* foreImage, bool IsDCare, bool IsDPed,vector<CvRect>& detectedObject,CvRect frameImg)
//{
//	//�Ƚ�ͼ��תΪ�Ҷ�ͼ���д���
//
//	if(NumofCars!=0)  //�����һ�εĳ��ļ��״̬
//	{
//		for(int i=0;i<NumofCars;i++)
//		{
//			delete AllHogInImg[i];
//		}
//		NumofCars=0;
//	}
//	if(NumofPeds!=0) //�����һ�ε��˵ļ��״̬
//	{
//		for(int i=0;i<NumofPeds;i++)
//		{
//			delete AllPedHogInImg[i];
//		}
//		NumofPeds=0;
//	}
//	PedHogInImg.clear();// ��һ�μ���hog�������
//
//	iW=DealImage->width;   //ȡ��ǰ����Ŀ�ĳߴ�
//	iH=DealImage->height;
//	iWeight=iW;
//	iHeight=iH;
//	IplImage*FgrayImg;  //RGBת��Ϊ�Ҷ�ͼ����
//	FgrayImg=cvCreateImage(cvSize(iW,iH),IPL_DEPTH_8U,1);
//	if(DealImage->nChannels==3)
//	{
//		cvCvtColor(DealImage, FgrayImg, CV_RGB2GRAY);	
//	}
//	else
//	{
//		cvCopy(DealImage,FgrayImg);
//	}
//
//	HogFeature*CurrentHog;   //��¼��ǰ��HoG����
//	int count_win=0;    //�Ա������ڽ��м�����
//    vector<CvSize2D32f>::iterator beg=SizeConversion.begin();  //��ȡ�任�ĳ߶�
//
//	while(beg!=SizeConversion.end())//��ÿ���߶��¶���������
//	{
//		int temptwid=Ped_N*beg->width;  //�߶�����
//		int temptheght=Ped_M*beg->height;
//		iW=double(iWeight)/beg->width;  //����֮���ͼ��߶�
//		iH=double(iHeight)/beg->height;
//		IplImage*grayImg;  //����֮��ĻҶ�ͼ��
//		grayImg=cvCreateImage(cvSize(iW,iH),IPL_DEPTH_8U,1);
//		cvResize(FgrayImg,grayImg,CV_INTER_LINEAR);  
////�����ڳ߶��µ�ͼ��DealImage����Ҫ������������� nwin_x*nwin_y 
//		int Car_nwin_x=floor(((iW-Car_N)/double(SkipStep))+1);    
//		int Car_nwin_y=floor(((iH-Car_M)/double(SkipStep))+1);
//		int Ped_nwin_x=floor(((iW-Ped_N)/double(SkipStep))+1);
//		int Ped_nwin_y=floor(((iH-Ped_M)/double(SkipStep))+1);
////���жϵ��˺ͳ��Ŀ�ĸ���		
//		int PedWin=Ped_nwin_y*Ped_nwin_y;
//		int WinInImg=Car_nwin_x*Car_nwin_y;   //һ��ͼ�ж�Ӧ������������win����Ŀ
//		//������м������һ��hogmap����ӳimg��Ӧ��hog����ͼ
//		//ͼ��Img��Ӧ���ܵ�Block��Ŀ  Car_Bnwin_x*Car_Bnwin_y
//		int Car_Bnwin_x=floor(double(iW-2*CellSz)/SkipStep+1);
//		int Car_Bnwin_y=floor(double(iH-2*CellSz)/SkipStep+1);
//
//		int SumOfBlocks=Car_Bnwin_y*Car_Bnwin_x;    //һ��ͼ�����е�block��������blocksize��block�ĳ��̶ܳȾ���
//
//		SubHogfeature*hogmap=new SubHogfeature[SumOfBlocks];   //����ͼ��
//		//����Gradient
//		CvMat*GradientX=cvCreateMat(iH,iW,CV_32FC1);
//		CvMat*GradientY=cvCreateMat(iH,iW,CV_32FC1);
//		CaculateGradient(grayImg,GradientX,GradientY);
//		//����������magnitudeֱ��ͼ
//		CvMat*GMag=cvCreateMat(iH,iW,CV_32FC1);
//		//���㷽��ͼXY
//		CvMat*YX=cvCreateMat(iH,iW,CV_32FC1);
//		int nAngle=20;
//        //����ȫ��hog����ѭ��
//		for(int i=0;i<iH;i++)
//		{
//			for(int j=0;j<iW;j++)
//			{
//				double tvalue=0;
//				double tvalue1=0;
//				tvalue=sqrt(cvmGet(GradientX,i,j)*cvmGet(GradientX,i,j)+cvmGet(GradientY,i,j)*cvmGet(GradientY,i,j));
//				if(tvalue==0)
//				{
//					tvalue=0.00001;
//				}
//				cvmSet(GMag,i,j,tvalue);
//				tvalue=double(cvmGet(GradientY,i,j))/cvmGet(GradientX,i,j);
//				tvalue1=(atan(tvalue)+3.1415926/2)*180/3.1415926;	//ת��Ϊ������	  
//				int temptdirect;
//				temptdirect=ceil(tvalue1/double(nAngle));
//				cvmSet(YX,i,j,temptdirect);
//			}
//		}
//		///////////////////////////////////��������ͼSubHogfeature*hogmap
//		for(int i=0;i<Car_Bnwin_y;i++)
//		{
//			for(int j=0;j<Car_Bnwin_x;j++)
//			{
//				int offsetx=j*SkipStep;
//				int offsety=i*SkipStep;
//				SubHogfeature t_feature;
//				for(int k=0;k<36;k++)
//					t_feature.hog36[k]=0;
//				double sumf=0;    //��һ�����ӣ���ÿһblock��Ҫ���й�һ��
//				for(int y=0;y<CellSz*2;y++)
//				{
//					for(int x=0;x<CellSz*2;x++)
//					{
//						int tempt=InWhichCell(y,x,CellSz);
//						int index=cvmGet(YX,y+offsety,x+offsetx);
//						double mag=cvmGet(GMag,y+offsety,x+offsetx);
//						t_feature.hog36[(tempt-1)*BinNum+index-1]+=mag;
//					}
//				}
//
//				for(int k=0;k<36;k++)
//				{
//					sumf+=t_feature.hog36[k]*t_feature.hog36[k];
//				}
//				double factor=sqrt(sumf);
//				for(int k=0;k<36;k++)
//				{
//					t_feature.hog36[k]=t_feature.hog36[k]/(factor+0.000001);
//				}
//				int tposition=0;
//				tposition=Car_Bnwin_x*i+j;
//				hogmap[tposition]=t_feature;
//			}
//		}
//		////������д��ڵ�����
//		//�������д����ҳ���
//
//		//�������д����ҳ�����,���Ը���ǰ���������,�����ǰ������Ĵ����ж�
//	
//
//		float*tempt_feature;
//		tempt_feature=new float[BlocksInWin*36];
//		//
//    if(IsDCare)
//	{
//		for(int i=0;i<Car_nwin_x;i++)
//			for(int j=0;j<Car_nwin_y;j++)
//			{
//				int offsetx=i*SkipStep;
//				int offsety=j*SkipStep;
//				int count_blocks=0;
//				for(int x=0;x<CarWin_Block_x;x++)
//					for(int y=0;y<CarWin_Block_y;y++)
//					{
//						int tposition=0;  
//						tposition=Car_Bnwin_x*(y+j)+x+i;
//						for(int k=0;k<36;k++)
//						{
//							int tempt=count_blocks*36+k;
//							tempt_feature[tempt]=hogmap[tposition].hog36[k];
//						}
//						count_blocks++;
//					}
//
//					int tempt1=0;
//					tempt1=CarClassifier->Classify(tempt_feature);
//					if(tempt1==1)
//					{
//						AllHogInImg[NumofCars]=new HogFeature;	
//						AllHogInImg[NumofCars]->x=float(offsetx)*beg->width;
//						AllHogInImg[NumofCars]->y=float(offsety)*beg->height;
//						AllHogInImg[NumofCars]->width=Car_N*beg->width;
//						AllHogInImg[NumofCars]->height=Car_M*beg->height;
//						AllHogInImg[NumofCars]->mid_x=AllHogInImg[NumofCars]->x+AllHogInImg[NumofCars]->width/2.0;
//						AllHogInImg[NumofCars]->mid_y=AllHogInImg[NumofCars]->y+AllHogInImg[NumofCars]->height/2.0;
//						if(NumofCars!=0)
//						{
//							AllHogInImg[NumofCars-1]->nextfeature=NumofCars;
//							AllHogInImg[NumofCars]->lastfeature=NumofCars-1;
//						}
//						else
//						{
//							AllHogInImg[NumofCars]->lastfeature=-1;
//						}
//						NumofCars++;	
//					}
//					count_win++;
//			}
//		}
//	if(IsDPed)
//	{
//			for(int i=0;i<Ped_nwin_x;i++)
//				for(int j=0;j<Ped_nwin_y;j++)
//				{
//					int offsetx=i*SkipStep;
//					int offsety=j*SkipStep;
//					int count_blocks=0;
//					for(int x=0;x<PedWin_Block_x;x++)
//						for(int y=0;y<PedWin_Block_y;y++)
//						{
//							int tposition=0;  
//							tposition=Car_Bnwin_x*(y+j)+x+i;
//							for(int k=0;k<36;k++)
//							{
//								int tempt=count_blocks*36+k;
//								tempt_feature[tempt]=hogmap[tposition].hog36[k];
//							}
//							count_blocks++;
//						}
//
//						int tempt1=0; //tempt_feature�ǵ�ǰ���������ڵ�hog����
//						tempt1=PedClassifier->Classify(tempt_feature);  //���з���
//
//						if((tempt1==1)&&(CaculateForeGRate(foreImage,cvRect(float(offsetx)*beg->width,float(offsety)*beg->height,temptwid,temptheght))>0.4))  //���Ϊ1��Ϊ��
//						{
//							AllPedHogInImg[NumofPeds]=new HogFeature;	
//							AllPedHogInImg[NumofPeds]->x=float(offsetx)*beg->width;
//							AllPedHogInImg[NumofPeds]->y=float(offsety)*beg->height;
//							AllPedHogInImg[NumofPeds]->width=temptwid;
//							AllPedHogInImg[NumofPeds]->height=temptheght;
//							AllPedHogInImg[NumofPeds]->mid_x=AllPedHogInImg[NumofPeds]->x+AllPedHogInImg[NumofPeds]->width/2.0;
//							AllPedHogInImg[NumofPeds]->mid_y=AllPedHogInImg[NumofPeds]->y+AllPedHogInImg[NumofPeds]->height/2.0;
//							if(NumofPeds!=0)
//							{
//								AllPedHogInImg[NumofPeds-1]->nextfeature=NumofPeds;
//								AllPedHogInImg[NumofPeds]->lastfeature=NumofPeds-1;
//							}
//							else
//							{
//								AllPedHogInImg[NumofPeds]->lastfeature=-1;
//							}
//							NumofPeds++;	
//			
//						}
//				}
//	}
//			//�ͷ���ʱʹ�õ���Դ
//				beg++;
//				cvReleaseImage(&grayImg);
//				cvReleaseMat(&GradientX);
//				cvReleaseMat(&GradientY);
//				cvReleaseMat(&YX);
//				cvReleaseMat(&GMag);
//				delete tempt_feature;
//				delete hogmap;
//	}
//	if(NumofCars!=0)
//		AllHogInImg[NumofCars-1]->nextfeature=-1;  //���һ��NumofCars��ʶ
//	if(NumofPeds!=0)
//		AllPedHogInImg[NumofPeds-1]->nextfeature=-1;  //���һ��NumofCars��ʶ
//	cvReleaseImage(&FgrayImg);
// 	NumOfFeatures=count_win;
//	int countfindout=0;
//	countfindout=MeltBox(NumofPeds,AllPedHogInImg,VMELX,VMELY,foreImage); //���ҵ����˽����ں�
//	NumofPeds=countfindout;
//
//	for(int i=0;i<NumofPeds;i++)
//	{
//		if(AllPedHogInImg[i]!=NULL)
//		{
//			CvRect temptR;
//			temptR.x=AllPedHogInImg[i]->x+frameImg.x;
//			temptR.y=AllPedHogInImg[i]->y+frameImg.y;
//			temptR.width=AllPedHogInImg[i]->width;
//			temptR.height=AllPedHogInImg[i]->height;
//			detectedObject.push_back(temptR);
//		}
//	}    
//	
//	return NumofCars+NumofPeds;
//}

int CCarPedDetector::MeltBox(int numofobjects, HogFeature** objectfeature, double ValueForMelt_x, double ValueForMelt_y, IplImage* ForeBackImage)
{
	int newNumObjects=0;
	int notypes=1;    //�����Ӧ�ĸ������
    for(int i=0;i!=numofobjects;i++)     //�Ը���
	{
		if(objectfeature[i]->notype==0)
		{
			objectfeature[i]->notype=notypes;
			notypes++;
		}
		for(int j=i+1;j!=numofobjects;j++)
		{

			if((abs(objectfeature[i]->mid_x-objectfeature[j]->mid_x)<ValueForMelt_x)&&(abs(objectfeature[i]->mid_y-objectfeature[j]->mid_y)<ValueForMelt_y))
			{
				objectfeature[j]->notype=objectfeature[i]->notype;
			}
			
		}
	}
	for(int i=0;i!=numofobjects;i++)
	{
		CvRect temptrect;
		temptrect.height= objectfeature[i]->height;
		temptrect.width = objectfeature[i]->width;
		temptrect.x     = objectfeature[i]->x;
		temptrect.y     = objectfeature[i]->y;
		if(CaculateForeGRate(ForeBackImage,temptrect)<0.36)
		{
			objectfeature[i]->notype=-1;
			objectfeature[i]->GoodOrNot=false;
		}
	}
	int notruetypes=0;
	for(int i=1;i<=notypes;i++)
	{

		double sumofx=0;
		double sumofy=0;
		double sumofwidth=0;
		double sumofheight=0;
		double count_n=0;
		for(int j=0;j!=numofobjects;j++)
		{
			if(objectfeature[j]->notype==i)
			{
				sumofx+=objectfeature[j]->x;
				sumofy+=objectfeature[j]->y;
				sumofwidth+=objectfeature[j]->width;
				sumofheight+=objectfeature[j]->height;
				count_n++;
			}     
		}
		if(count_n!=0)
		{
			HogFeature*newhog;
			newhog=new HogFeature;
			newhog->GoodOrNot=true;
			newhog->x=double(sumofx)/count_n;
			newhog->y=double(sumofy)/count_n;
			newhog->width=double(sumofwidth)/count_n;
			newhog->height=double(sumofheight)/count_n;
			delete objectfeature[notruetypes];
			objectfeature[notruetypes]=newhog;
			notruetypes++;
		}
	}
	for(int i=notruetypes;i!=numofobjects;i++)
	{
		delete objectfeature[i];
	}

	return notruetypes;

}

// �����Ӧ�Ŀ��ǰ������
double CCarPedDetector::CaculateForeGRate(IplImage* ForeGround, CvRect& objectblob)
{
	char*tempt_position;
	char*dataorign=ForeGround->imageDataOrigin;
	int f_widthStep=ForeGround->widthStep;
	int numofallpixel=0;
	int forpixel=0;
	for(int j=objectblob.y;j<(objectblob.y+objectblob.height);j++)
	{
		for(int i=objectblob.x;i<(objectblob.x+objectblob.width);i++)
		{
			tempt_position=dataorign+i+j*f_widthStep;
			if((*tempt_position)!=0)
			{
				forpixel++;
			}
			numofallpixel++;
		}
	}
	double rate;
	rate=double(forpixel)/numofallpixel;
	return rate;
	return 0;
}
//��ǳ������������minAscend������Ŀ��С��possiblePwidth
int CCarPedDetector::StatisticTheXPixel(IplImage* ForeGround,IplImage*orignImage,int minAscend,int possiblePwidth)  //ͳ��2ֵǰ��ͼƬ��X�᷽���ϵ�ֱ��ͼ��Ϣ
{
	int WiofF=ForeGround->width;
	int HiofF=ForeGround->height;
	IplImage*ShowResultImage,*LBImage;
	ShowResultImage=cvCreateImage(cvGetSize(ForeGround),8,1);
	LBImage=cvCreateImage(cvGetSize(ForeGround),8,1);
	unsigned char*dataorign=(unsigned char*)ShowResultImage->imageDataOrigin;
	memset(dataorign,0,ForeGround->widthStep*ForeGround->height);
	dataorign=(unsigned char*)LBImage->imageDataOrigin;
	memset(dataorign,0,ForeGround->widthStep*ForeGround->height);
    unsigned char*tempt=NULL;
	dataorign=(unsigned char*)ForeGround->imageDataOrigin;
	int*XAxis=NULL;
	int*DXAxis=NULL;
	int*DerAxis01=NULL;
	int*Acme=NULL;
    DerAxis01=new int[WiofF]();
	XAxis=new int[WiofF]();
    DXAxis=new int[WiofF]();
	Acme=new int[WiofF]();
	for(int i=0;i<HiofF;i++)
	{
		for(int j=0;j<WiofF;j++)
		{
			tempt=dataorign+(ForeGround->widthStep*i)+j;
			if(*tempt>125)
			{
				XAxis[j]++;
			}
		}
	}
	for(int i=0;i<WiofF;i++)
	{
	//	cvDrawLine(ShowResultImage,cvPoint(i,HiofF-1),cvPoint(i,HiofF-XAxis[i]),CV_RGB(255,255,255),1);
	    if(i>=2&&(i<=WiofF-3))
		{
            DXAxis[i]=XAxis[i-2]*0.12+XAxis[i-1]*0.18+XAxis[i]*0.4+XAxis[i+1]*0.18+XAxis[i+2]*0.12;
		}
		else
		{
			DXAxis[i]=XAxis[i];
		}
	//	cvDrawLine(LBImage,cvPoint(i,HiofF-1),cvPoint(i,HiofF-DXAxis[i]),CV_RGB(255,255,255),1);
	}
	int temptacme=0;
	int count=0;
    int localmax=0,localmin=0;
	int localmaxindex=0,localminindex=0;
	//������Ϊ���˵Ĳ���
	for(int i=0;i<WiofF-1;i++)
	{
		localmax=DXAxis[i];
		for(int j=i+1;(j<WiofF)&&(j<i+possiblePwidth/2);j++)
		{
          if(DXAxis[j]>localmax)
		  {
			  localmax=DXAxis[j];
			  localmaxindex=j;
		  }
		}
		localmin=localmax;
		for(int j=localmaxindex;(j<WiofF)&&(j<i+possiblePwidth);j++)
		{
		  if(DXAxis[j]<localmin)
		  {
				localmin=DXAxis[j];
				localminindex=j;
		  }
		}
		int t=MIN(abs(localmax-DXAxis[i]),abs(localmax-localmin));
		if(t>minAscend)
		{
			Acme[count]=localmaxindex;
			count++;
			i=localminindex;
		}

	}
    //  for(int i=0;i<count;i++)
	//{
	//	cvDrawLine(LBImage,cvPoint(Acme[i],HiofF-1),cvPoint(Acme[i],0),CV_RGB(255,255,255),1);
	//}
	
	//��XAxisֱ��ͼ���н��Ƹ�˹�˲�ģ��Ϊ5������0.12  0.18 0.4 0.18 0.12 
	//cvNamedWindow("LBXStati");
	//cvShowImage("LBXStati",LBImage);
	cvReleaseImage(&ShowResultImage);
	cvReleaseImage(&LBImage);
	delete XAxis;
	delete DXAxis;
	delete DerAxis01;
	delete Acme;

     return count;

}

double CCarPedDetector::FindPedObject(IplImage* DealImage, IplImage* foreImage,vector<CvRect>& detectedObject,CvRect frameImg,CvSize WFrameSize,double SizeAlpha)
{
	//���ȸ���ǰ����Ϣ���Ƴ��˵Ŀ��ܵĺ�����
	vector<int>AcmeX;
    int NofPed=StatisticTheXPixelandH(foreImage,DealImage,10,20,AcmeX);
    CvRect LocalRect;  //��dealimage���پֲ�����
	CvRect ChangeRect;
	//���ݺ�����͹��Ƶ��˵Ĵ�ų߶���С������Χ��ֻ�Կ��еĲ��ֽ������˼��
	for(int i=0;i<NofPed;i++)
	{
		LocalRect.x=(AcmeX[i]-PedHalfSearchWidth>0?AcmeX[i]-PedHalfSearchWidth:0);
		LocalRect.width=(AcmeX[i]+PedHalfSearchWidth<(DealImage->width-1)?2*PedHalfSearchWidth:(DealImage->width-LocalRect.x));
		if(LocalRect.width<PedHalfSearchWidth)
		{
			i++;
			continue;
		}
		unsigned char*dataorign=(unsigned char*)foreImage->imageDataOrigin;
		unsigned char*tempt=NULL;
		int top=0,bottom=0;
		int realtop=0,realbottom=0;
		for(int j=0;j<foreImage->height;j++)
		{
			tempt=dataorign+j*foreImage->widthStep+AcmeX[i];
            if(*tempt>125)
			{
				realtop=j;
				top=(j-2>=0?j-2:j);
				break;
			}
		}
		for(int j=foreImage->height-1;j>0;j--)
		{
			tempt=dataorign+j*foreImage->widthStep+AcmeX[i];
			if(*tempt>125)
			{
				realbottom=j;
				bottom=(j+5<=foreImage->height-1?j+5:j);
				break;
			}
		}
		if((top-bottom)>0)
		{
			continue;
		}
       LocalRect.y=top;
	   LocalRect.height=bottom-top;
   
	   int y1=bottom+frameImg.y+8;
	   int y11=y1*SizeAlpha;
	   int y2=top+frameImg.y-5;
	   int y22=y2*SizeAlpha;
	   int y3=(y2+y1)/2.0;
	   int y33=y3*SizeAlpha;
	   int realy=realbottom-realtop;

	   if((realy<y11+15&&realy>y22-15)||(SizeAlpha<0)||(SizeAlpha>1))
	   {
		       ChangeRect.x=LocalRect.x+frameImg.x;
		   	   ChangeRect.y=LocalRect.y+frameImg.y;
		   	   ChangeRect.width=LocalRect.width;
		   	   ChangeRect.height=LocalRect.height;
		       detectedObject.push_back(ChangeRect); 
		//	   SizeAlpha=OnlineLearnSAlpha(ChangeRect.y+ChangeRect.height,ChangeRect.height,baseN,SizeTrainN,SizeAlpha,15);
			   continue;
	   }
	   CvRect TopPPed;
	   TopPPed.x=((AcmeX[i]-y11/4.0)>0?AcmeX[i]-y11/4.0:0);
	   TopPPed.width=y11/2.0;
	   if((TopPPed.x+TopPPed.width)>(DealImage->width-1))
	   {
			TopPPed.width=DealImage->width-1-TopPPed.x;
	   }
	   TopPPed.y=realbottom-y11;
	   if(TopPPed.y<0)
	   {
		   TopPPed.y=0;
	   }
	   TopPPed.height=y11;
	   if((TopPPed.y+TopPPed.height)>DealImage->height-1)
	   {
            TopPPed.height=DealImage->height-1-TopPPed.y;
	   }
	   if(TopPPed.height>0&&TopPPed.width>0)
	   {
		  cvSetImageROI(DealImage,TopPPed);
		  IplImage*temptImage=cvCreateImage(cvGetSize(DealImage),IPL_DEPTH_8U,1);
		  cvCopy(DealImage,temptImage);   //��ȡǰ����ĻҶ�ͼ
		  cvResetImageROI(DealImage);
		  if(IsImagePed(temptImage)==1)
		  {
		     ChangeRect.x=TopPPed.x+frameImg.x;
		     ChangeRect.y=TopPPed.y+frameImg.y;
		     ChangeRect.width=TopPPed.width;
		     ChangeRect.height=TopPPed.height;
		     detectedObject.push_back(ChangeRect); 
		  }
		  cvReleaseImage(&temptImage);
	   }
	   TopPPed.x=((AcmeX[i]-y33/4.0)>0?AcmeX[i]-y33/4.0:0);
	   TopPPed.width=y33/2.0-2;
	   if((TopPPed.x+TopPPed.width)>(DealImage->width-1))
	   {
		   TopPPed.width=DealImage->width-1-TopPPed.x;
	   }
	   TopPPed.y=top;
	   if(TopPPed.y<0)
	   {
		   TopPPed.y=0;
	   }
	   TopPPed.height=y33;
	   if((TopPPed.y+TopPPed.height)>DealImage->height-1)
	   {
		   TopPPed.height=DealImage->height-1-TopPPed.y;
	   }
	   if(TopPPed.height>0&&TopPPed.width>0)
	   {
		  cvSetImageROI(DealImage,TopPPed);
		  IplImage*temptImage=cvCreateImage(cvGetSize(DealImage),IPL_DEPTH_8U,1);
		  cvCopy(DealImage,temptImage);   //��ȡǰ����ĻҶ�ͼ
		  cvResetImageROI(DealImage);
		  if(IsImagePed(temptImage)==1)
		  {
			  ChangeRect.x=TopPPed.x+frameImg.x;
			  ChangeRect.y=TopPPed.y+frameImg.y;
			  ChangeRect.width=TopPPed.width;
			  ChangeRect.height=TopPPed.height;
			  detectedObject.push_back(ChangeRect); 
		  }
		  cvReleaseImage(&temptImage);
	   }

  //   ChangeRect.x=LocalRect.x+frameImg.x;
	 //ChangeRect.y=LocalRect.y+frameImg.y;
  //   ChangeRect.width=LocalRect.width;
  //   ChangeRect.height=LocalRect.height;
  //   detectedObject.push_back(ChangeRect); 

	}

	//���ݸ������λ�ú�sizealpha����ͳ����Ϣ�����˵Ĵ�ų߶�

	return SizeAlpha;
}

int CCarPedDetector::StatisticTheXPixelandH(IplImage* ForeGround, IplImage* orignImage, int minAscend, int possiblePwidth,vector<int>& AcmeX)
{
	int WiofF=ForeGround->width;
	int HiofF=ForeGround->height;
	IplImage*ShowResultImage,*LBImage;
	ShowResultImage=cvCreateImage(cvGetSize(ForeGround),8,1);
	LBImage=cvCreateImage(cvGetSize(ForeGround),8,1);
	unsigned char*dataorign=(unsigned char*)ShowResultImage->imageDataOrigin;
	memset(dataorign,0,ForeGround->widthStep*ForeGround->height);
	dataorign=(unsigned char*)LBImage->imageDataOrigin;
	memset(dataorign,0,ForeGround->widthStep*ForeGround->height);
	unsigned char*tempt=NULL;
	dataorign=(unsigned char*)ForeGround->imageDataOrigin;
	int*XAxis=NULL;
	int*DXAxis=NULL;
	XAxis=new int[WiofF]();
	DXAxis=new int[WiofF]();
	for(int i=0;i<HiofF;i++)
	{
		for(int j=0;j<WiofF;j++)
		{
			tempt=dataorign+(ForeGround->widthStep*i)+j;
			if(*tempt>125)
			{
				XAxis[j]++;
			}
		}
	}
	for(int i=0;i<WiofF;i++)
	{
		cvDrawLine(ShowResultImage,cvPoint(i,HiofF-1),cvPoint(i,HiofF-XAxis[i]),CV_RGB(255,255,255),1);
		if(i>=2&&(i<=WiofF-3))
		{
			DXAxis[i]=XAxis[i-2]*0.12+XAxis[i-1]*0.18+XAxis[i]*0.4+XAxis[i+1]*0.18+XAxis[i+2]*0.12;
		}
		else
		{
			DXAxis[i]=XAxis[i];
		}
		cvDrawLine(LBImage,cvPoint(i,HiofF-1),cvPoint(i,HiofF-DXAxis[i]),CV_RGB(255,255,255),1);
	}
	int temptacme=0;
	int count=0;
	int localmax=0,localmin=0;
	int localmaxindex=0,localminindex=0;
	//������Ϊ���˵Ĳ���
	for(int i=0;i<WiofF-1;i++)
	{
		localmax=DXAxis[i];
		for(int j=i+1;(j<WiofF)&&(j<i+possiblePwidth/2);j++)
		{
			if(DXAxis[j]>localmax)
			{
				localmax=DXAxis[j];
				localmaxindex=j;
			}
		}
		localmin=localmax;
		for(int j=localmaxindex;(j<WiofF)&&(j<i+possiblePwidth);j++)
		{
			if(DXAxis[j]<localmin)
			{
				localmin=DXAxis[j];
				localminindex=j;
			}
		}
		int t=MIN(abs(localmax-DXAxis[i]),abs(localmax-localmin));
		if(t>minAscend)
		{
			AcmeX.push_back(localmaxindex);
			i=localminindex;
		}

	}
	for(int i=0;i<AcmeX.size();i++)
	{
		cvDrawLine(LBImage,cvPoint(AcmeX[i],HiofF-1),cvPoint(AcmeX[i],0),CV_RGB(255,255,255),1);
	}

	//��XAxisֱ��ͼ���н��Ƹ�˹�˲�ģ��Ϊ5������0.12  0.18 0.4 0.18 0.12 
	//cvNamedWindow("LBXStati");
	//cvShowImage("LBXStati",LBImage);
	cvReleaseImage(&ShowResultImage);
	cvReleaseImage(&LBImage);
	delete XAxis;
	delete DXAxis;
	return AcmeX.size();
}

