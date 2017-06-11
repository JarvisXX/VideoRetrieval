#include "clude.h"
#include "Monitor.h"
#include "math.h"
#include "item.h"

extern int item_id;
extern int video_id;
extern int item_sum;
extern vector<item> item_list;
extern bool is_new_video;

CvScalar* FakeRGB;
int	checkID;		//要观察的目标ID 

vector<PathTraj> path;
vector<PathTraj> Grouppath[5];
Monitor*monitors[5] = { NULL, NULL, NULL, NULL, NULL };   //4路视频对应4个监视器    
float resizeFactor;		//要观察的目标ID 

Monitor::Monitor()
	: moveout_OPEN(false)
	, ilHandle(1)
	, TESTPathCount(0)
{
	m_pFG = new MoGFGDetector();		//创建高斯前景检测器
	m_FGTrainFrames = 30;
	m_FrameCount = 0;
	m_pCF = new CornerFinder();			//创建角点检测器
	m_pTR = new MFTracker();

	SuddenAccelerateValue = 0;
	//用伪彩色显示某些数据的大小
	FakeRGB = new CvScalar[10];
	FakeRGB[0] = CV_RGB(0, 0, 170);
	FakeRGB[1] = CV_RGB(255, 170, 0);
	FakeRGB[2] = CV_RGB(0, 85, 255);
	FakeRGB[3] = CV_RGB(255, 255, 0);
	FakeRGB[4] = CV_RGB(0, 255, 255);
	FakeRGB[5] = CV_RGB(85, 255, 170);
	FakeRGB[6] = CV_RGB(170, 255, 85);
	FakeRGB[7] = CV_RGB(0, 170, 255);
	FakeRGB[8] = CV_RGB(0, 255, 0);
	FakeRGB[9] = CV_RGB(255, 85, 0);
	m_frame = NULL;
}
Monitor::~Monitor()
{
	delete m_pFG;

	delete m_pCF;

	delete FakeRGB;


	delete m_pTR;
	//delete moveoutact;


	if (m_frame != NULL)
		cvReleaseImage(&m_frame);

	Grouppath[ilHandle].erase(Grouppath[ilHandle].begin(), Grouppath[ilHandle].end());

	//delete m_pTR;
}

void Monitor::Init(int ww, int hh, int stride, float factor)
{
	factor = 2;
	resizeFactor = 2;
	iW = (int)(ww / factor);
	iH = (int)(hh / factor);
	m_frame = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 3); //创建缩小后的图像
	iStride = m_frame->widthStep;   //新图像的行宽
	iLen = iStride * iH;            //新图像总的字节数
	m_f = factor;                   //存储缩小因子
	for (int i = 0; i<10; i++)
	{
		CurrentWarnInfo[i] = 0;
	}
	Crosslineflag = 0;
	Crosslinex1 = 0;
	Crosslinex2 = 0;
	Crossliney1 = 0;
	Crossliney2 = 0;

	//初始化模块
	m_pCF->initialize(iW, iH);      //角点检测模块的初始化
	m_pFG->initialize(iW, iH, iStride, m_FGTrainFrames);	  //高斯混合模型的初始化，这里首先完成空间的分配 m_FGTrainFrames计数	
	m_pTR->initialize(iW, iH);        //跟踪算法模块初始化
	//GetRightTime();
	//初始化报警链表WarningString
	//StartOfWarning=NULL;
	//EndOfWarning=NULL;
	//m_warningString..clear();

	carpedDec = new CCarPedDetector;
	carpedDec->InitializeDetector();
}

int ImgClass(Mat image){
	int classNum = 10;
	int lowH, highH, lowS, highS, lowV, highV;
	vector<double> ptRate(classNum, 0.0);
	string outClass;

	for (int ic = 0; ic < classNum; ic++)
	{
		switch (ic)
		{
		case 0://blue  
			lowH = 100;
			highH = 124;
			lowS = 63;
			highS = 255;
			lowV = 76;
			highV = 220;
			break;
		case 1://orange  
			lowH = 11;
			highH = 25;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 2://yellow  
			lowH = 22;
			highH = 37;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 3:
			lowH = 35;
			highH = 77;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 4:
			lowH = 125;
			highH = 155;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 5:
			lowH = 0;
			highH = 10;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 6://white  
			lowH = 0;
			highH = 180;
			lowS = 0;
			highS = 25;
			lowV = 225;
			highV = 255;
			break;
		case 7://grey  
			lowH = 0;
			highH = 180;
			lowS = 28;
			highS = 40;
			lowV = 30;
			highV = 221;
			break;
		case 8://black  
			lowH = 0;
			highH = 180;
			lowS = 0;
			highS = 255;
			lowV = 0;
			highV = 30;
			break;
		case 9://red  
			lowH = 156;
			highH = 180;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		}

		Mat imgHSV;
		vector<Mat> hsvSplit;
		cvtColor(image, imgHSV, COLOR_BGR2HSV);

		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);

		Mat imgThresholded;
		inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThresholded);

		int nonZeroNum = 0;
		vector<Mat> channelsImg;
		split(imgThresholded, channelsImg);
		Mat_<uchar> imgResult = channelsImg[0];
		for (int ia = 0; ia < imgResult.rows; ia++)
			for (int ib = 0; ib < imgResult.cols; ib++)
				if (imgResult(ia, ib) != 0)
					nonZeroNum++;

		double rateCac = (double)nonZeroNum / (double)(imgResult.rows*imgResult.cols);
		ptRate[ic] = rateCac;
	}

	double curRate = 0.0;
	int classN;
	for (int id = 0; id < ptRate.size(); id++)
	{
		if (ptRate[id] > curRate)
		{
			curRate = ptRate[id];
			classN = id;
		}
	}

	/*
	switch (classN){
	case 0:
		outClass = "blue";
		break;
	case 1:
		outClass = "orange";
		break;
	case 2:
		outClass = "yellow";
		break;
	case 3:
		outClass = "green";
		break;
	case 4:
		outClass = "violet";
		break;
	case 5:
		outClass = "red";
		break;
	case 6:
		outClass = "white";
		break;
	case 7:
		outClass = "grey";
		break;
	case 8:
		outClass = "black";
		break;
	case 9:
		outClass = "red";
	}
	*/

	return classN;
}

IplImage*  Monitor::Process(IplImage* frame, rule*EntityRule, long lHandle)
{
	m_FrameCount++;
	cvResize(frame, m_frame, CV_INTER_NN);    //缩小帧的尺寸,提高处理速度
	ilHandle = lHandle;
	m_pFG->process(m_frame, lHandle);	//前景提取
	if (m_pFG->ifStable())		//只有当 前景提取模块 完成背景学习时 才继续后续操作
	{
		
		m_pFGMask = m_pFG->getFG();	//取前景mask
		m_pBG = m_pFG->getBG();			//取背景（被程序中没有用）

		/*	前景	
		IplImage*fore = cvCreateImage(cvGetSize(m_frame), m_frame->depth, m_frame->nChannels);
		char image_name[13];
		sprintf(image_name, "mask.jpg");
		cvSaveImage(image_name, m_pBG);
		//sprintf(image_name, "fore.jpg");
		//cvSaveImage(image_name, frame);
		cvAbsDiff(m_frame, m_pBG, fore);
		sprintf(image_name, "%05d%s", m_FrameCount, ".jpg");
		cvSaveImage(image_name, m_frame);

		sprintf(image_name, "%05d%s", m_FrameCount, "fore.jpg");
		cvSaveImage(image_name, fore);
		*/
	

		m_BlobList = m_pFG->getBlobList();	//取blob list

		m_pCF->findCorner(m_frame, m_pFGMask);//角点检测
		//	m_pCF->findCorner(m_frame,m_pFGMask);//角点检测

		
		m_pTR->track(m_frame, m_pFGMask, m_pBG, m_BlobList, m_pCF->getFeatureMap());	//主跟踪程序
		//从MFTracker类中 取跟踪到的目标的list,画框输出在图像上
		vector<myObject*>& trackList = *(m_pTR->getTrackList());

		//输出帧数
		// ofstream fout("D:/S.txt", ios::app);
		// fout << m_FrameCount << endl;
		// fout.close();

		PathDetection(frame, m_pFGMask, trackList, EntityRule);
		CrossLineDetection(frame, trackList, EntityRule, Crosslinex1, Crossliney1, Crosslinex2, Crossliney2);
	}
	//背景模型的跟新 放在最后，前面的m_pFG->process(m_frame)只做前景检测，如此可以根据当前跟踪到的物体的位置，避免更新消去停下来的目标
	
	m_pFG->update(m_pTR->getTrackLabel(), m_pTR->getUnstableLabel());
	return frame;
}
int delRedundantPointFromTailofVector(vector<CvPoint> &src)
{
	int RedundantNum = 0;
	int size = src.size();
	if (size <= 1) return 0;
	CvPoint lastPoint = src[size - 1];
	for (int i = size - 2; i >= 0; i--)
	{
		CvPoint thisPoint = src[i];
		if (thisPoint.x == lastPoint.x&&thisPoint.y == lastPoint.y)
		{
			lastPoint = thisPoint;
			RedundantNum++;
		}
		else
		{
			thisPoint.x == lastPoint.x;
			break;
		}
	}
	return RedundantNum;
}
void Monitor::PathDetection(IplImage *frame, IplImage *m_pFGMask, std::vector<myObject*> &trackList, rule *pRule)
{
	for (int i = 0; i < (int)trackList.size(); ++i)
	{
		if (trackList[i]->ID > 0)
		{

			//cout << &trackList[i]->blob.medianX << endl;
			// speedup
			CutBlobRec(frame, m_pFGMask, &trackList[i]->blob, trackList[i]->item_id_);
			DrawBlobRec(frame, &trackList[i]->blob);
			trackList[i]->catagories = vehicle;

		}

	}

	///////////////////每一帧开始时进行初始化////////////////////
	for (int i = 0; i < ObjectMatched.size(); i++)
	{
		ObjectMatched[i] = false;
	}
	///////////////////每一帧开始时进行初始化////////////////////
	//if (pRule->Rule_route.route_flag != 1) return;

	for (int i = 0; i < (int)trackList.size(); ++i)
	{
		if (trackList[i]->ID > 0)
		{
			int id = trackList[i]->ID;
			vector<CvPoint> *pTraj = trackList[i]->trajectory;

			//////////Joseph Yu///////////
			int tempID = trackList[i]->ID;
			int position = posInVector(myObjectID, tempID);
			if (position == -1)
			{
				myObjectID.push_back(tempID);
				itemIDlist.push_back(trackList[i]->item_id_);
				ObjectMatched.push_back(true);	//新目标，不需要匹配
				allTrajectories.push_back(*pTraj);
			}
			else
			{
				allTrajectories[position] = *pTraj;
				ObjectMatched[position] = true;
			}
			//////////Joseph Yu///////////
			// Draw Line
			/*
			CvScalar rgb = FakeRGB[id % 10];
			for (int j = pTraj->size() - 1; j > 0; j--)
			{
				cvLine(frame, cvPoint((*pTraj)[j].x * m_f, (*pTraj)[j].y * m_f), cvPoint((*pTraj)[j - 1].x * m_f, (*pTraj)[j - 1].y * m_f), rgb, 1);
			}
			*/
		}
	}
	//////////Joseph Yu///////////
	//每一帧结尾时，判断有无目标没有匹配，没有被匹配的目标的轨迹被视为完整轨迹
	for (int i = 0; i < ObjectMatched.size(); i++)
	{
		if (ObjectMatched[i] == false)
		{
			int position = posInVector(completedObjPos, i);
			if (position == -1)
			{

				int RedundantNum = delRedundantPointFromTailofVector(allTrajectories[i]);
				vector<CvPoint> temp;
				temp.resize(allTrajectories[i].size() - RedundantNum);
				copy(allTrajectories[i].begin(), allTrajectories[i].end() - RedundantNum, temp.begin());
				if (temp.size()>60)//控制最短的轨迹长度，避免过短的轨迹。一条新轨迹。
				{
					completedObjPos.push_back(i);
					itemIDlist_detect.push_back(itemIDlist[i]);
					completeTrajectories.push_back(temp);//用于画图的轨迹集合，保存了所有的完整轨迹，不清空，聚类训练时用的是这个集合里的轨迹
					completeTrajectories_detect.push_back(temp);//用于检测异常的轨迹集合，每次检测完后要清空，避免检测过的轨迹再次被检测

				}

			}
		}
	}
	//////////Joseph Yu///////////

	//
	//here 

	//

	//for(int i = 0;i < (int)trackList.size();++i)
	//{
	//	if(trackList[i]->ID > 0)
	//	{
	//          TESTPathCount++;
	//	//   if(TESTPathCount>101)
	//       //   AfxMessageBox("haha");		
	//		CvRect rect;
	//		rect = trackList[i]->blob.outRect;
	//		int id = trackList[i]->ID;
	//		CvScalar rgb = FakeRGB[id % 10];
	//		cvRectangle(frame, cvPoint(rect.x * m_f, rect.y * m_f),
	//			cvPoint(rect.x * m_f + rect.width * m_f, rect.y * m_f + rect.height * m_f),rgb ,2);
	//		
	//		if(trackList[i]->stateSeq->size() >= 2)
	//		{
	//		    int num = trackList[i]->stateSeq->size();
	//			if((*(trackList[i]->stateSeq))[num-2] == obj_tracked && (*(trackList[i]->stateSeq))[num-1] == obj_missing)
	//			{
	//                   	
	//				PathTraj newPath;
	//				//newPath.trj = trackList[i]->trajectory;
	//				newPath.lastTime = 60;
	//				newPath.ID = trackList[i]->ID;
	//    			Grouppath[ilHandle].push_back(newPath);
	//
	//			}
	//		}
	//	}
	//}

	//for(int i = 0;i < (int)Grouppath[ilHandle].size();++i)
	//{
	//	if(Grouppath[ilHandle][i].lastTime > 0)
	//	{
	//		int id = Grouppath[ilHandle][i].ID;
	//		vector<CvPoint> *pTraj = Grouppath[ilHandle][i].trj;
	//		CvScalar rgb = FakeRGB[id % 10];
	//		for(int j = pTraj->size()-1; j > 0; j--)
	//		{
	//			cvLine(frame, cvPoint( (*pTraj)[j].x * m_f, (*pTraj)[j].y * m_f ), cvPoint( (*pTraj)[j-1].x * m_f, (*pTraj)[j-1].y * m_f), rgb, 2);
	//		}
	//		Grouppath[ilHandle][i].lastTime--;
	//	}
	//	else if(Grouppath[ilHandle][i].lastTime == 0)
	//	{
	//		Grouppath[ilHandle].erase(Grouppath[ilHandle].begin() + i);
	//	}
	//}

	//级联异常
	if (completeTrajectories_detect.size() > 0) {
		string base = basedir_global;
		if (readFlag == false)
		{
			int size1 = 0, size2 = 0;
			string filename_size_v = "centers_v_size.txt";
			ifstream os_size_in((base + filename_size_v).c_str());
			os_size_in >> size1 >> size2;
			os_size_in.close();

			string filename_cen_v = "centers_v.db";
			ifstream is((base + filename_cen_v).c_str(), ios::binary | ios::in);
			vector<double> tra(size2, 0);//下层vector也要开辟内存才行
			vector<vector<double>> centers_in(size1, tra);
			centers_v_read = centers_in;//保存读入的聚类顶点
			//is.read((char*)&centers_read[0], size1 * size2*sizeof(double));
			for (int i = 0; i < size1; i++)
			{
				is.read((char*)&centers_v_read[i][0], size2*sizeof(double));
			}
			//cout << "速度特征聚类centers:" << endl;
			//for (int i = 0; i < size1; i++)
			//{
			//	for (int j = 0; j < size2; j++)
			//		cout << centers_v_read[i][j] << " ";
			//	cout << endl;
			//}
			//system("pause");
			is.close();


			int size1_traj = 0, size2_traj = 0;
			string filename_size_traj = "centers_traj_size.txt";
			ifstream os_size_in_traj((base + filename_size_traj).c_str());
			os_size_in_traj >> size1_traj >> size2_traj;
			os_size_in_traj.close();

			string filename_cen_traj = "centers_traj.db";
			ifstream is_traj((base + filename_cen_traj).c_str(), ios::binary | ios::in);
			vector<double> tra_traj(size2_traj, 0);//下层vector也要开辟内存才行
			vector<vector<double>> centers_in_traj(size1_traj, tra_traj);
			centers_traj_read = centers_in_traj;//保存读入的聚类顶点
			//is.read((char*)&centers_read[0], size1 * size2*sizeof(double));
			for (int i = 0; i < size1_traj; i++)
			{
				is_traj.read((char*)&centers_traj_read[i][0], size2_traj*sizeof(double));
			}

			//cout << "轨迹特征聚类centers:" << endl;
			//for (int i = 0; i < size1_traj; i++)
			//{
			//	for (int j = 0; j < size2_traj; j++)
			//		cout << centers_traj_read[i][j] << " ";
			//	cout << endl;
			//}
			is_traj.close();

			readFlag = true;
		}

		//每次对completeTrajectories里的完整轨迹提取特征，检测并画出轨迹之后（两种颜色），要将completeTrajectories清空
		vector<vector<double>> FeatAll_v;//临时计算特征用
		vector<vector<double>> FeatAll_traj;
		double w[5];
		//w[0] for 平均速度幅值
		//w[1] for 平均速度向量
		//w[2] for 平均位置
		//w[3] for 起点终点向量
		//w[4] for 轨迹描述点
		w[0] = 1; w[1] = 1; w[2] = 1; w[3] = 1; w[4] = 1;
		GetFeatureAllRespective(completeTrajectories_detect, w, FeatAll_v, FeatAll_traj);
		//检测
		/*
		*训练1.avi：参数width_v = 1,thresh_v = 3e-3,width_traj=0.0001, thresh_traj=3e-2聚类的到的中心，threshold_v=100,threshold=220可以使训练1.avi轨迹全部正常

		*/
		//原来的threshold_v = 100 threshold_traj = 220;
		double threshold_v = 1.6;//初步的调试想法：将聚类训练的视频拿来测试，先取一个较小的threshhold，如果发生异常就增大threshold，直到刚好没有异常
		int returnValue_v = 1;
		double threshold_traj = 190;//初步的调试想法：将聚类训练的视频拿来测试，先取一个较小的threshhold，如果发生异常就增大threshold，直到刚好没有异常
		int returnValue_traj = 1;

		for (int i = 0; i < FeatAll_v.size(); ++i)//FeatAll_v和FeatAll_traj的size应该一样
		{
			char info[90];
			//detectCount++;
			sprintf(info, "正在检测第%d条完整轨迹......", itemIDlist_detect[i]);
			string infoStr(info);
			cout << endl;
			cout << infoStr << endl;
			cout << "<<<<<<<<<<<<<<<<<<结果是：>>>>>>>>>>>>>>>>>>>>" << endl;
			returnValue_v = detect_func_one_traj(centers_v_read, FeatAll_v[i], threshold_v);
			returnValue_traj = detect_func_one_traj(centers_traj_read, FeatAll_traj[i], threshold_traj);

			if (returnValue_v == 1)
			{
				isNormalAll_v.push_back(1);
				item_list[itemIDlist_detect[i]].is_abnormal_v = 0;
				//isNormalAll_plot.push_back(1);
			}
			else if (returnValue_v == 0)
			{
				isNormalAll_v.push_back(0);
				item_list[itemIDlist_detect[i]].is_abnormal_v = 1;
				//isNormalAll_plot.push_back(0);
				cout << "检测到速度异常！" << endl;
			}
			if (returnValue_traj == 1)
			{
				isNormalAll_traj.push_back(1);
				item_list[itemIDlist_detect[i]].is_abnormal_traj = 0;
				//isNormalAll_plot.push_back(1);
			}
			else if (returnValue_traj == 0)
			{
				isNormalAll_traj.push_back(0);
				item_list[itemIDlist_detect[i]].is_abnormal_traj = 1;
				//isNormalAll_plot.push_back(0);
				cout << "检测到轨迹异常！" << endl;
			}

			if (returnValue_v == 0 || returnValue_traj == 0)
			{
				isNormalAll_plot.push_back(0);//异常
				isNormalAll_res.push_back(0);
				//isNormal = false;
			}
			else
			{
				isNormalAll_plot.push_back(1);//正常
				isNormalAll_res.push_back(1);
			}
		}

		//double threshold_traj = 220;//初步的调试想法：将聚类训练的视频拿来测试，先取一个较小的threshhold，如果发生异常就增大threshold，直到刚好没有异常
		//int returnValue_traj = 1;
		//for (int i = 0; i < FeatAll_traj.size(); ++i)
		//{
		//	returnValue_traj = detect_func_one_traj(centers_traj_read, FeatAll_traj[i], threshold_traj);
		//	if (returnValue_traj == 1)
		//	{
		//		isNormalAll_traj.push_back(1);
		//		//isNormalAll_plot.push_back(1);
		//	}
		//	else if (returnValue_traj == 0)
		//	{
		//		isNormalAll_traj.push_back(0);
		//		//isNormalAll_plot.push_back(0);
		//		cout << "检测到轨迹异常！" << endl;
		//	}
		//}

		completeTrajectories_detect.erase(completeTrajectories_detect.begin(), completeTrajectories_detect.end());//清空已经检测的轨迹，避免重复检测
		vector<vector<CvPoint>>().swap(completeTrajectories_detect);
		itemIDlist_detect.erase(itemIDlist_detect.begin(), itemIDlist_detect.end());
		vector<int>().swap(itemIDlist_detect);
	}

	//检测时画图
	// speedup
	/*
	if (completeTrajectories.size() > 0 && isNormalAll_plot.size() > 0)
	{
		for (int i = 0; i < completeTrajectories.size(); i++)//检测时画图用completeTrajectories
		{
			CvScalar rgb = FakeRGB[isNormalAll_plot[i] + 1 % 100];//颜色选取可能要更改一下
			for (int j = 1; j < completeTrajectories[i].size(); j++)
			{
				cvLine(frame, cvPoint(completeTrajectories[i][j].x * m_f, completeTrajectories[i][j].y * m_f), cvPoint(completeTrajectories[i][j - 1].x * m_f, completeTrajectories[i][j - 1].y * m_f), rgb, 2);
			}
		}
	}

	if (isNormalAll_plot.size() >= 10)//画图的轨迹数太多时清空以前的轨迹，避免影响显示效果
	{
		isNormalAll_plot.erase(isNormalAll_plot.cbegin(), isNormalAll_plot.cend());
		vector<int>().swap(isNormalAll_plot);

		completeTrajectories.erase(completeTrajectories.begin(), completeTrajectories.end());
		vector<vector<CvPoint>>().swap(completeTrajectories);
	}
	*/
}
//将每一帧中的目标裁剪出来，并判断类别，颜色记录下来。

int colorDetection(Mat image, Mat mask)
{
	int color;
	int classNum = 10;
	int lowH, highH, lowS, highS, lowV, highV;
	vector<double> ptRate(classNum, 0.0);
	string outClass;

	for (int ic = 0; ic < classNum; ic++) {
		switch (ic) {
		case 0://red
			lowH = 156;
			highH = 180;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 1://orange
			lowH = 11;
			highH = 25;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 2://yellow
			lowH = 22;
			highH = 37;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 3:
			lowH = 35;
			highH = 77;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 4:
			lowH = 125;
			highH = 155;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 5:
			lowH = 0;
			highH = 10;
			lowS = 43;
			highS = 255;
			lowV = 46;
			highV = 255;
			break;
		case 6://white  
			lowH = 0;
			highH = 180;
			lowS = 0;
			highS = 25;
			lowV = 225;
			highV = 255;
			break;
		case 7://grey  
			lowH = 0;
			highH = 180;
			lowS = 28;
			highS = 40;
			lowV = 30;
			highV = 221;
			break;
		case 8://black  
			lowH = 0;
			highH = 180;
			lowS = 0;
			highS = 255;
			lowV = 0;
			highV = 30;
			break;
		case 9://blue
			lowH = 100;
			highH = 124;
			lowS = 63;
			highS = 255;
			lowV = 76;
			highV = 220;
			break;
		}

		Mat imgHSV;
		vector<Mat> hsvSplit;
		cvtColor(image, imgHSV, COLOR_BGR2HSV);

		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);

		Mat imgThresholded;
		inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThresholded);
		int nonZeroNum = 0;
		//vector<Mat> channelsImg;
		//split(imgThresholded, channelsImg);
		//Mat_<uchar> imgResult = channelsImg[0];
		for (int ia = 0; ia < imgThresholded.rows; ia++)
			for (int ib = 0; ib < imgThresholded.cols; ib++)
				if (imgThresholded.at<uchar>(ia, ib) != 0 && mask.at<uchar>(ia, ib) != 0)
					nonZeroNum++;

		double rateCac = (double)nonZeroNum / (double)(imgThresholded.rows*imgThresholded.cols);
		ptRate[ic] = rateCac;
	}

	double curRate = 0.0;
	for (int id = 0; id < ptRate.size(); id++)
	{
		if (ptRate[id] > curRate)
		{
			curRate = ptRate[id];
			color = id;
		}
	}
	if (color == 0)
		color = 5;
	/*
	switch (classN) {
	case 0:
	outClass = "red";
	break;
	case 1:
	outClass = "orange";
	break;
	case 2:
	outClass = "yellow";
	break;
	case 3:
	outClass = "green";
	break;
	case 4:
	outClass = "violet";
	break;
	case 5:
	outClass = "red";
	break;
	case 6:
	outClass = "white";
	break;
	case 7:
	outClass = "grey";
	break;
	case 8:
	outClass = "black";
	break;
	case 9:
	outClass = "blue";
	}
	*/
	return color;
}

void Monitor::CutBlobRec(IplImage* img, IplImage *m_pFGMask, myBlob* pBlob, int &item_id_)
{
	CvRect rect = pBlob->outRect;   //获取跟踪目标的跟踪外框
	CvRect rect2 = cvRect(rect.x * m_f, rect.y * m_f, rect.width * m_f, rect.height * m_f);
	int id = pBlob->ID;
	string cutid;
	string cutframe;
	string cuttext;
	string statistictext;
	int catagory = 0;
	//int color = 1;

	stringstream ss;
	ss << id;
	ss >> cutid;
	stringstream ss2;
	ss2 << m_FrameCount;
	ss2 >> cutframe;
	cuttext = "cut\\" + cutframe + "_" + cutid + ".jpg";
	statistictext = cutframe + "," + cutid;
	//cuttext =  cutframe + "_" + cutid + ".jpg";

	//resize m_pFGMask
	IplImage *tmp_mask = cvCreateImage(cvSize(m_pFGMask->width*m_f, m_pFGMask->height*m_f), m_pFGMask->depth, m_pFGMask->nChannels);
	IplImage *FGMask;
	cvResize(m_pFGMask, tmp_mask, CV_INTER_AREA);
	cvSetImageROI(tmp_mask, rect2);
	FGMask = cvCreateImage(cvSize(tmp_mask->roi->width, tmp_mask->roi->height), tmp_mask->depth, tmp_mask->nChannels);
	cvCopy(tmp_mask, FGMask);
	//cvShowImage("mask", mask);

	//save ROI
	const char* name = cuttext.c_str();
	IplImage* ori;
	ori = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
	cvCopy(img, ori);
	IplImage* result;
	cvSetImageROI(ori, rect2);
	// speedup
	result = cvCreateImage(cvSize(ori->roi->width, ori->roi->height), ori->depth, ori->nChannels);
	cvCopy(ori, result);
	//cvSaveImage(name, ori);
	//result=cvLoadImage(name,1);
	cvResetImageROI(ori);

	//catogory
	double possibility;
	possibility = carpedDec->IsImagePed(result);
	
	if (possibility<-25) {
		cout << possibility << endl;
		catagory = 0;
	}
	else if (rect.width > 0.6*rect.height)
			catagory = 0;//vehicle
		else
			catagory = 1; //pedstrain

	
	// 颜色识别
	int color_f;
	color_f = colorDetection(Mat(result), Mat(FGMask));

	ofstream fout("statistic.txt", ios::app);
	fout << statistictext << "," << catagory << "," << color_f << endl;
	fout.close();
	
	// item data
	if (id == 1 && is_new_video) {
		item_sum = item_id;
		is_new_video = FALSE;
	}

	item_id = item_sum + id;
	item_id_ = item_id;

	if (item_id < item_list.size()) {
		item_list[item_id].color_list[color_f] += 1;
		item_list[item_id].end_frame = m_FrameCount;
	}
	else {
		item item_;
		while (item_id >= item_list.size()) {
			item_list.push_back(item_);
		}
		item_list[item_id].id = id;
		item_list[item_id].category = catagory;
		item_list[item_id].video_id = video_id;
		item_list[item_id].color_list[color_f] += 1;
		item_list[item_id].color = color_f;
		item_list[item_id].start_frame = m_FrameCount;
		item_list[item_id].end_frame = m_FrameCount;
	}

	cvReleaseImage(&tmp_mask);
	cvReleaseImage(&FGMask);
	cvReleaseImage(&ori);
	cvReleaseImage(&result);
}
///在img图上画bolb框
void Monitor::DrawBlobRec(CvArr* img, myBlob* pBlob)

{
	/*CvRect rect = pBlob->outRect;   //获取跟踪目标的跟踪外框
	int id = pBlob->ID;             //跟踪目标的ID号
	CvScalar rgb = FakeRGB[id % 10];//根据id产生不同颜色
	fstream fout("D:/S.txt", ios::app);
	fout << id << " " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;
	fout.close();
	cvRectangle(img, cvPoint(rect.x * m_f, rect.y * m_f),
		cvPoint(rect.x * m_f + rect.width * m_f, rect.y * m_f + rect.height * m_f),
		rgb, 1);
		*/
	CvRect rect = pBlob->outRect;   //获取跟踪目标的跟踪外框
	int id = pBlob->ID;
	char idtext[25];
	_itoa(id, idtext, 10);
	//跟踪目标的ID号
	CvScalar rgb = FakeRGB[id % 10];//根据id产生不同颜色

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.618, 0.618, 0, 1, 8);
	cvPutText(img, idtext, cvPoint(rect.x * m_f, rect.y * m_f), &font, rgb);
	cvRectangle(img, cvPoint(rect.x * m_f, rect.y * m_f),
		cvPoint(rect.x * m_f + rect.width * m_f, rect.y * m_f + rect.height * m_f),
		rgb, 1);

	fstream fout("D:/S.txt", ios::app);
	fout << id << " " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;
	fout.close();
}

void Monitor::DrawBlobRec(IplImage* img, myBlob* pBlob, int Warn)
{
	cout << "in ipl" << endl;
	CvRect rect = pBlob->outRect;
	
	if (Warn > 0)
	{
		cvRectangle(img, cvPoint(rect.x * m_f, rect.y * m_f),
			cvPoint(rect.x * m_f + rect.width * m_f, rect.y * m_f + rect.height * m_f),
			CV_RGB(255, 0, 0), 4);
	}
	else
	{
		cvRectangle(img, cvPoint(rect.x * m_f, rect.y * m_f),
			cvPoint(rect.x * m_f + rect.width * m_f, rect.y * m_f + rect.height * m_f),
			CV_RGB(128, 128, 128));
	}
}

//在图像img中作直线从（xa，ya）到（xb，yb）
void Monitor::DrawLine(IplImage *img, float xa, float xb, float ya, float yb, int lineid, int type)
{
	CvScalar*scalar[3];
	scalar[0] = new CvScalar;
	*scalar[0] = cvScalar(255, 0, 0);
	scalar[1] = new CvScalar;
	*scalar[1] = cvScalar(0, 255, 0);
	scalar[2] = new CvScalar;
	*scalar[2] = cvScalar(0, 0, 255);
	cvLine(img, cvPoint(xa, ya), cvPoint(xb, yb), *scalar[type], 2, 0);
	char num[5];
	sprintf(num, "%d", lineid);
	CvFont*font = new CvFont;
	*font = cvFont(1.0);
	cvPutText(img, num, cvPoint((xa + xb) / 2, (ya + yb) / 2), font, cvScalar(255, 0, 0));
	for (int i = 0; i<3; i++)
	{
		delete scalar[i];
	}
	delete font;
}
//在img图像中画出目标块pBlob

void Monitor::CrossLineDetection(IplImage *frame, std::vector<myObject*> &trackList, rule *pRule, int x1, int x2, int y1, int y2)
{
	int linex1, linex2, liney1, liney2;
	linex1 = x1;
	liney1 = x2;
	linex2 = y1;
	liney2 = y2;


	bool isduedirect = 0;
	bool isintoout = 0;
	bool isoutin = 0;
	if (!Crosslineflag) return;
	char*typeofcross[3] = { "BiLineCross", "OutWardLineCross", "InWardLineCross" };
	CvFont*font = new CvFont;
	*font = cvFont(1.0);
	int countl = 1;       //过线的条数
	for (int i = 0; i < countl; i++)     //绘制拌线
	{
		DrawLine(frame, linex1, liney1,
			linex2, liney2, i + 1, 0);
	}

	for (int i = 0; i < (int)trackList.size(); i++)   //分析场景中的跟踪目标
	{
		int max_n, indx;
		max_n = trackList[i]->Warn[0];   //难道说warn0对应第一条拌线？
		indx = 0;
		for (int n = 1; n < countl; ++n)
		{
			if (max_n <= trackList[i]->Warn[n])
			{
				max_n = trackList[i]->Warn[n];
				indx = n;  //拌线的标记？
			}
		}
		if (max_n > 0)
		{
			cvPutText(frame, "LineCross", cvPoint(trackList[i]->blob.outRect.x * m_f, trackList[i]->blob.outRect.y * m_f), font, cvScalar(255, 0, 0));
			DrawBlobRec(frame, &trackList[i]->blob, trackList[i]->Warn[indx]);
		}
		else
		{
			//DrawBlobRec(frame, &trackList[i]->blob,0);
		}
	}

	for (int i = 0; i < (int)trackList.size(); ++i)
	{
		for (int m = 0; m < countl; ++m)
		{
			trackList[i]->Warn[m] = trackList[i]->Warn[m] - (trackList[i]->Warn[m] > 0);
		}

		if (trackList[i]->ID > 0)
		{
			float xb = trackList[i]->cx;
			float yb = trackList[i]->cy + trackList[i]->blob.outRect.height / 2;
			xb = xb * m_f;
			yb = yb * m_f;
			for (int j = 0; j < countl; ++j)
			{
				int DirectionFlag = 0;
				if (DirectionFlag == 0)
				{
					trackList[i]->lineCur[j] = CrossDirection(xb, yb, linex1, linex2, liney1, liney2);
					if (((trackList[i]->linePre[j] == Left && trackList[i]->lineCur[j] == Right)
						|| (trackList[i]->linePre[j] == Right && trackList[i]->lineCur[j] == Left)) && IsPointOnLine(xb, yb, linex1, linex2, liney1, liney2))
					{
						cout << "拌线！" << endl;
						isduedirect = 1;
						trackList[i]->Warn[j] = 10;
					}
					trackList[i]->linePre[j] = trackList[i]->lineCur[j];
				}
				
			}
		}
	}
	CurrentWarnInfo[7] = isduedirect;
	CurrentWarnInfo[8] = isintoout;
	CurrentWarnInfo[9] = isoutin;
	delete font;
}
bool Monitor::IsInArea(double TopP_X[20], double TopP_Y[20], int NumP, double k[20], double X, double Y)
{
	int i, count = 0;
	double Tx, Ty;
	for (i = 0; i<NumP - 1; i++)
	{
		if (k[i] == 1000)
		{
			if ((TopP_X[i] - X)>0 && (Y - TopP_Y[i])*(Y - TopP_Y[i + 1])<0)
				count++;
		}
		else
		{
			Ty = Y;
			Tx = TopP_X[i] - double(TopP_Y[i] - Ty) / k[i];
			if ((Tx - X)>0 && (Y - TopP_Y[i])*(Y - TopP_Y[i + 1])<0)
				count++;
		}
	}

	if (count % 2 == 0)
		return false;
	else
		return true;
}
linedirection Monitor::CrossDirection(float x, float y, float x1, float y1, float x2, float y2)
{
	y = -y; y1 = -y1; y2 = -y2;
	if (x1 == x2)
	{
		if (x >= x1) { return Right; }
		else { return Left; }
	}
	else
	{
		float k = (y1 - y2) / (x1 - x2);
		float b = x1 - y1 / k;
		float b1 = x - y / k;
		if (b1 <= b) { return Left; }
		else { return Right; }
	}
}
bool Monitor::IsPointOnLine(float x, float y, float x1, float y1, float x2, float y2)
{
	double Top_x[4];
	double Top_y[4];
	double op_x[4];
	double op_y[4];
	int wide = 15;
	bool outcome = false;
	bool outcome1 = false;
	Top_x[0] = x1 - wide;
	Top_y[0] = y1;
	Top_x[1] = x1 + wide;
	Top_y[1] = y1;
	Top_x[2] = x2 + wide;
	Top_y[2] = y2;
	Top_x[3] = x2 - wide;
	Top_y[3] = y2;
	op_x[0] = x1;
	op_y[0] = y1 + wide;
	op_x[1] = x1;
	op_y[1] = y1 - wide;
	op_x[2] = x2;
	op_y[2] = y2 - wide;
	op_x[3] = x2;
	op_y[3] = y2 + wide;
	double k[4];
	double k1[4];
	CalculateK(Top_x, Top_y, 4, k);
	outcome = IsInArea(Top_x, Top_y, 4, k, x, y);
	CalculateK(op_x, op_y, 4, k1);
	outcome1 = IsInArea(op_x, op_y, 4, k1, x, y);
	return outcome || outcome1;
}
void Monitor::CalculateK(double TopP_X[20], double TopP_Y[20], int NumP, double *k)
{
	int i;
	for (i = 0; i<NumP - 1; i++)
	{
		if (TopP_X[i + 1] != TopP_X[i])
			k[i] = double(TopP_Y[i + 1] - TopP_Y[i]) / double(TopP_X[i + 1] - TopP_X[i]);
		else
			k[i] = 1000;
	}
}
template <typename T>
int Monitor::posInVector(const vector<T> &vec, const T elem)
{
	int sizeOfVec = vec.size();
	if (sizeOfVec == 0) return -1;
	for (int i = 0; i < sizeOfVec; i++)
	{
		if (vec[i] == elem) return i;
	}
	return -1;
}

Points Monitor::Crosslinelocation(IplImage *frame)
{
	Points TEMP;
	IplImage* R = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	IplImage* G = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	IplImage* B = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	cvSplit(frame, B, G, R, NULL);

	Mat I = frame;
	Mat r = R;
	Mat g = G;
	Mat b = B;
	
	/* for (int i = 0; i<r.rows; i++)
	{
	for (int j = 0; j < r.cols; j++)
	{
	if (r.at<uchar>(i, j) <100|| b.at<uchar>(i, j) >120|| g.at<uchar>(i, j) >120)

	r.at<uchar>(i, j) = 0;
	}
	}*/

	for (int i = 0; i<r.rows; i++)
	{
		for (int j = 0; j < r.cols; j++)
		{
			if (r.at<uchar>(i, j) >80 && b.at<uchar>(i, j) <80 && g.at<uchar>(i, j) <80)

				r.at<uchar>(i, j) = 255;
			else
				r.at<uchar>(i, j) = 0;

		}
	}

	Mat image = frame;

	//Mat contours;
	//Canny(r, contours, 125, 350);
	//imshow("c", contours);
	vector<Vec4i> lines;
	// 检测直线，最小投票为90，线条不短于50，间隙不小于10 
	HoughLinesP(r, lines, 1, CV_PI / 180, 10, 100, 3);
	//vector<Vec4i>::const_iterator it=lines.begin(); 
	double length = 0;
	int x1;
	int y1;
	int x2;
	int y2;
	for (size_t i = 0; i < lines.size(); i++)//size_t即unsigned int
	{
		Vec4i k = lines[i];//此处为英文字母l
		if ((k[2] - k[0])*(k[2] - k[0]) + (k[3] - k[1])* (k[3] - k[1]) > length)
		{
			x1 = k[0];
			y1 = k[1];
			x2 = k[2];
			y2 = k[3];
			length = (k[2] - k[0])*(k[2] - k[0]) + (k[3] - k[1])* (k[3] - k[1]);
		}

	}
	cout << x1 << "  " << y1 << "  " << x2 << "  " << y2 << "  " << endl;
	TEMP.points[0] = x1;
	TEMP.points[1] = y1;
	TEMP.points[2] = x2;
	TEMP.points[3] = y2;

	return TEMP;
}
