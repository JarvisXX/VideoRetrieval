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
	nextfeature=-2;   //下一个索引初始化为-1
	lastfeature=-2;
	features=NULL;
	notype=0;
	
}
HogFeature::~HogFeature(void)
{
	if(features!=NULL)
		delete features;
}
void HogFeature::SetDim(int d)   //设定features
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
	Ped_M=128;     //人的搜索框尺寸 M*N
	Ped_N=64;
	Car_M=64;      //车的搜索框尺寸 M*N
	Car_N=128;
	SkipStep=8;    //block每一次移动像素  ,实际是重复步长 ，因为在img中产生的block必然是重复的
	//BlockSkipStep=8;
	CellSz=8;      //cell对应的尺度 cellsz*cellsz
	BlockSz=16;   //我们采用一个Block中包含4个Cell的结构，所以BlockSz=CellSz*2
	BinNum=9;     //方向数 ，我们采用9个
	Angle=180;    //180度，不考虑360度，因为背景跟前景的差度不明确
	FeatDim=BinNum*4;
	//初始化汽车分类器
	CarClassifier=new CStrongTreeClassifier;
	CarClassifier->InitialStrongClassifier(NUMOFWEAK,"weakteset.txt");
	CarClassifier->classvalue=6.5;      //?分类阈值
    //初始化人的分类器
	PedClassifier=new CStrongTreeClassifier;
	PedClassifier->InitialStrongClassifier(PEDNUMOFWEAK,"pedstrongclass.txt");
	PedClassifier->classvalue=10;        //行人分类阈值
 //   CarClassifier->TestReadWrite("K:\\weakteset1.txt");
	CarWin_Block_x=floor(double(Car_N-BlockSz)/SkipStep+1);
	CarWin_Block_y=floor(double(Car_M-BlockSz)/SkipStep+1);
	PedWin_Block_x=floor(double(Ped_N-BlockSz)/SkipStep+1);
	PedWin_Block_y=floor(double(Ped_M-BlockSz)/SkipStep+1);
	PedBlocksInWin=PedWin_Block_x*PedWin_Block_y;

	BlocksInWin=CarWin_Block_x*CarWin_Block_y;  //一个搜索框win中所包含的blcok数目
	for(int i=0;i<100;i++)
	{
		CarNo[i]=-1;
	}


}
void CCarPedDetector::CaculateGradient(IplImage* grayImage,CvMat*GradientX,CvMat*GradientY)
{
	CvMat*GRAY1,headgray;
	
	CvMat*GRAY=cvCreateMat(iH,iW,CV_32FC1);
	//cvInitMatHeader( GRAY, iH, iW, CV_32FC1 );//初始化

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
	 //计算GradientX
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
//计算GradientY
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

// 判断y,x是在哪一个cell
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

// 将距离近的任意方框进行融合，返回最终得到的物体个数，以及新得到的hogfeature,
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
//		int min_i=-1;  //记录距离最短的两个物体
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
//将检测到的人，车进行融合
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

// 判断给出灰度图片是人的概率，根据hog特征判断的话
double CCarPedDetector::IsImagePed(IplImage* WaitImg)
{
	iW=WaitImg->width;
	iH=WaitImg->height;
	int iWeight=iW;
	int iHeight=iH;
	IplImage*FgrayImg;
	double possible_result=0;
	FgrayImg=cvCreateImage(cvSize(iW,iH),IPL_DEPTH_8U,1);

	//如果待处理图像不是灰度图则先转换成灰度图处理
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
	int Ped_nwin_x=1;    //在图像DealImage中需要处理的搜索框数 nwin_x*nwin_y 
	int Ped_nwin_y=1;

	int PedWin=1;
	//这里进行计算求出一张hogmap，反映img对应的hog特征图
	//图像Img对应的总的Block数目  Bnwin_x*Bnwin_y
	int Bnwin_x=floor(double(Ped_N-2*CellSz)/SkipStep+1);
	int Bnwin_y=floor(double(Ped_M-2*CellSz)/SkipStep+1);
	int SumOfBlocks=Bnwin_y*Bnwin_x;    //一张图中所有的block数，根据blocksize和block的稠密程度决定
	SubHogfeature*hogmap=new SubHogfeature[SumOfBlocks];   //特征图！
	//计算Gradient
	CvMat*GradientX=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	CvMat*GradientY=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	CaculateGradient(grayImg,GradientX,GradientY);
	// speedup
	//cvNamedWindow("s");
	//cvShowImage("s", grayImg);
	//计算向量的magnitude直方图
	CvMat*GMag=cvCreateMat(Ped_M,Ped_N,CV_32FC1);
	//计算方向图XY
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
			tvalue1=(atan(tvalue)+3.1415926/2)*180/3.1415926;	//转化为度数	  
			int temptdirect;
			temptdirect=ceil(tvalue1/double(nAngle));
			cvmSet(YX,i,j,temptdirect);
		}
	}
	///////////////////////////////////计算haar特征图SubHogfeature*hogmap
	for(int i=0;i<Bnwin_y;i++)
	{
		for(int j=0;j<Bnwin_x;j++)
		{
			int offsetx=j*SkipStep;
			int offsety=i*SkipStep;
			SubHogfeature t_feature;
			for(int k=0;k<36;k++)
				t_feature.hog36[k]=0;
			double sumf=0;    //归一化因子，对每一block都要进行归一化
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
	////求出所有窗口的特征
	//遍历所有窗口找出人
	//遍历所有窗口找出车子
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

			//释放临时使用的资源
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
//	//先将图像转为灰度图进行处理
//
//	if(NumofCars!=0)  //清除上一次的车的检测状态
//	{
//		for(int i=0;i<NumofCars;i++)
//		{
//			delete AllHogInImg[i];
//		}
//		NumofCars=0;
//	}
//	if(NumofPeds!=0) //清除上一次的人的检测状态
//	{
//		for(int i=0;i<NumofPeds;i++)
//		{
//			delete AllPedHogInImg[i];
//		}
//		NumofPeds=0;
//	}
//	PedHogInImg.clear();// 上一次检测的hog特征清楚
//
//	iW=DealImage->width;   //取当前处理的块的尺寸
//	iH=DealImage->height;
//	iWeight=iW;
//	iHeight=iH;
//	IplImage*FgrayImg;  //RGB转换为灰度图像处理
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
//	HogFeature*CurrentHog;   //记录当前的HoG特征
//	int count_win=0;    //对遍历窗口进行计数！
//    vector<CvSize2D32f>::iterator beg=SizeConversion.begin();  //获取变换的尺度
//
//	while(beg!=SizeConversion.end())//在每个尺度下都进行搜索
//	{
//		int temptwid=Ped_N*beg->width;  //尺度缩放
//		int temptheght=Ped_M*beg->height;
//		iW=double(iWeight)/beg->width;  //缩放之后的图像尺度
//		iH=double(iHeight)/beg->height;
//		IplImage*grayImg;  //缩放之后的灰度图像
//		grayImg=cvCreateImage(cvSize(iW,iH),IPL_DEPTH_8U,1);
//		cvResize(FgrayImg,grayImg,CV_INTER_LINEAR);  
////在现在尺度下的图像DealImage中需要处理的搜索框数 nwin_x*nwin_y 
//		int Car_nwin_x=floor(((iW-Car_N)/double(SkipStep))+1);    
//		int Car_nwin_y=floor(((iH-Car_M)/double(SkipStep))+1);
//		int Ped_nwin_x=floor(((iW-Ped_N)/double(SkipStep))+1);
//		int Ped_nwin_y=floor(((iH-Ped_M)/double(SkipStep))+1);
////待判断的人和车的框的个数		
//		int PedWin=Ped_nwin_y*Ped_nwin_y;
//		int WinInImg=Car_nwin_x*Car_nwin_y;   //一张图中对应的所有搜索框win的数目
//		//这里进行计算求出一张hogmap，反映img对应的hog特征图
//		//图像Img对应的总的Block数目  Car_Bnwin_x*Car_Bnwin_y
//		int Car_Bnwin_x=floor(double(iW-2*CellSz)/SkipStep+1);
//		int Car_Bnwin_y=floor(double(iH-2*CellSz)/SkipStep+1);
//
//		int SumOfBlocks=Car_Bnwin_y*Car_Bnwin_x;    //一张图中所有的block数，根据blocksize和block的稠密程度决定
//
//		SubHogfeature*hogmap=new SubHogfeature[SumOfBlocks];   //特征图！
//		//计算Gradient
//		CvMat*GradientX=cvCreateMat(iH,iW,CV_32FC1);
//		CvMat*GradientY=cvCreateMat(iH,iW,CV_32FC1);
//		CaculateGradient(grayImg,GradientX,GradientY);
//		//计算向量的magnitude直方图
//		CvMat*GMag=cvCreateMat(iH,iW,CV_32FC1);
//		//计算方向图XY
//		CvMat*YX=cvCreateMat(iH,iW,CV_32FC1);
//		int nAngle=20;
//        //计算全部hog特征循环
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
//				tvalue1=(atan(tvalue)+3.1415926/2)*180/3.1415926;	//转化为度数？	  
//				int temptdirect;
//				temptdirect=ceil(tvalue1/double(nAngle));
//				cvmSet(YX,i,j,temptdirect);
//			}
//		}
//		///////////////////////////////////计算特征图SubHogfeature*hogmap
//		for(int i=0;i<Car_Bnwin_y;i++)
//		{
//			for(int j=0;j<Car_Bnwin_x;j++)
//			{
//				int offsetx=j*SkipStep;
//				int offsety=i*SkipStep;
//				SubHogfeature t_feature;
//				for(int k=0;k<36;k++)
//					t_feature.hog36[k]=0;
//				double sumf=0;    //归一化因子，对每一block都要进行归一化
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
//		////求出所有窗口的特征
//		//遍历所有窗口找出人
//
//		//遍历所有窗口找出车子,可以根据前景点做提高,避免非前景区域的错误判断
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
//						int tempt1=0; //tempt_feature是当前的搜索窗口的hog特征
//						tempt1=PedClassifier->Classify(tempt_feature);  //进行分类
//
//						if((tempt1==1)&&(CaculateForeGRate(foreImage,cvRect(float(offsetx)*beg->width,float(offsety)*beg->height,temptwid,temptheght))>0.4))  //如果为1则为人
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
//			//释放临时使用的资源
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
//		AllHogInImg[NumofCars-1]->nextfeature=-1;  //最后一个NumofCars标识
//	if(NumofPeds!=0)
//		AllPedHogInImg[NumofPeds-1]->nextfeature=-1;  //最后一个NumofCars标识
//	cvReleaseImage(&FgrayImg);
// 	NumOfFeatures=count_win;
//	int countfindout=0;
//	countfindout=MeltBox(NumofPeds,AllPedHogInImg,VMELX,VMELY,foreImage); //对找到的人进行融合
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
	int notypes=1;    //聚类对应的各个类别
    for(int i=0;i!=numofobjects;i++)     //对各个
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

// 计算对应的框的前景比率
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
//标记出波峰的落差大于minAscend，波峰的宽度小于possiblePwidth
int CCarPedDetector::StatisticTheXPixel(IplImage* ForeGround,IplImage*orignImage,int minAscend,int possiblePwidth)  //统计2值前景图片在X轴方向上的直方图信息
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
	//检测可能为行人的波峰
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
	
	//对XAxis直方图进行近似高斯滤波模板为5个像素0.12  0.18 0.4 0.18 0.12 
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
	//首先根据前景信息估计出人的可能的横坐标
	vector<int>AcmeX;
    int NofPed=StatisticTheXPixelandH(foreImage,DealImage,10,20,AcmeX);
    CvRect LocalRect;  //在dealimage中再局部处理
	CvRect ChangeRect;
	//根据横坐标和估计的人的大概尺度缩小搜索范围，只对框中的部分进行行人检测
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
		  cvCopy(DealImage,temptImage);   //获取前景块的灰度图
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
		  cvCopy(DealImage,temptImage);   //获取前景块的灰度图
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

	//根据给定框的位置和sizealpha加上统计信息估计人的大概尺度

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
	//检测可能为行人的波峰
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

	//对XAxis直方图进行近似高斯滤波模板为5个像素0.12  0.18 0.4 0.18 0.12 
	//cvNamedWindow("LBXStati");
	//cvShowImage("LBXStati",LBImage);
	cvReleaseImage(&ShowResultImage);
	cvReleaseImage(&LBImage);
	delete XAxis;
	delete DXAxis;
	return AcmeX.size();
}

