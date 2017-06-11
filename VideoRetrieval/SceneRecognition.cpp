#include "clude.h"


//void screnere_trian()
//{
//	char name[3][20] = { "office", "passage", "stairs" };
//	cout << name[0] << endl;
//	int ImgWidht = 120;
//	int ImgHeight = 120;
//	vector<string> img_path;
//	vector<int> img_catg;
//	int nLine = 0;
//	string buf;
//	ifstream svm_data("F:/最终图片/路径centrist.txt");
//	unsigned long n;
//
//	while (svm_data)
//	{
//		if (getline(svm_data, buf))
//		{
//			nLine++;
//			if (nLine <= 120)
//			{
//				img_catg.push_back(0);
//				img_path.push_back(buf);//图像路径 
//			}
//			else if (nLine > 436 && nLine <= 556)
//			{
//				img_catg.push_back(1);
//				img_path.push_back(buf);//图像路径 
//			}
//			else if (nLine > 895 && nLine <= 1100)
//			{
//				img_catg.push_back(2);
//				img_path.push_back(buf);//图像路径 
//			}
//
//
//
//		}
//	}
//	svm_data.close();//关闭文件  
//
//
//
//	Mat data_mat, res_mat;
//	int nImgNum = nLine;            //读入样本数量  
//	////样本矩阵，nImgNum：横坐标是样本数量， WIDTH * HEIGHT：样本特征向量，即图像大小  
//	//data_mat = Mat::zeros( nImgNum, 12996, CV_32FC1 );    
//	//类型矩阵,存储每个样本的类型标志  
//	res_mat = Mat::zeros(nImgNum, 1, CV_32FC1);
//
//	Mat src;
//	Mat trainImg = Mat::zeros(ImgHeight, ImgWidht, CV_8UC3);//需要分析的图片  
//
//	for (string::size_type i = 0; i != img_path.size(); i++)
//	{
//		src = imread(img_path[i].c_str(), 1);
//
//		cout << " processing " << img_path[i].c_str() << endl;
//
//		resize(src, trainImg, cv::Size(ImgWidht, ImgHeight), 0, 0, INTER_CUBIC);
//		HOGDescriptor *hog = new HOGDescriptor(cvSize(ImgWidht, ImgHeight), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
//		vector<float>descriptors;//结果数组     
//		hog->compute(trainImg, descriptors, Size(1, 1), Size(0, 0)); //调用计算函数开始计算
//		if (i == 0)
//		{
//			data_mat = Mat::zeros(nImgNum, descriptors.size(), CV_32FC1); //根据输入图片大小进行分配空间 
//		}
//		cout << "dims: " << descriptors.size() << endl;
//		n = 0;
//		for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++)
//		{
//			data_mat.at<float>(i, n) = *iter;
//			n++;
//		}
//		//cout<<SVMtrainMat->rows<<endl;  
//		res_mat.at<float>(i, 0) = img_catg[i];
//		cout << " end processing " << img_path[i].c_str() << " " << img_catg[i] << endl;
//	}
//
//	CvSVM svm;
//	CvSVMParams param;
//	CvTermCriteria criteria;
//	criteria = cvTermCriteria(CV_TERMCRIT_EPS, 1000, FLT_EPSILON);
//	param = CvSVMParams(CvSVM::C_SVC, CvSVM::RBF, 10.0, 0.09, 1.0, 10.0, 0.5, 1.0, NULL, criteria);
//
//	/*
//	SVM种类：CvSVM::C_SVC
//	Kernel的种类：CvSVM::RBF
//	degree：10.0（此次不使用）
//	gamma：8.0
//	coef0：1.0（此次不使用）
//	C：10.0
//	nu：0.5（此次不使用）
//	p：0.1（此次不使用）
//	然后对训练数据正规化处理，并放在CvMat型的数组里。
//	*/
//	//☆☆☆☆☆☆☆☆☆(5)SVM学习☆☆☆☆☆☆☆☆☆☆☆☆         
//	svm.train(data_mat, res_mat, Mat(), Mat(), param);
//	//☆☆利用训练数据和确定的学习参数,进行SVM学习☆☆☆☆     
//	svm.save("F:/江鹤/SVM_DATA.xml");
//
//}
extern CvSVM svm;
string screnere_test(Mat frametest)
{
	char name[3][20] = { "office", "passage", "stairs" };
	//CvSVM svm;
	
	int ImgWidht = 120;
	int ImgHeight = 120;

	//检测样本  
	/*vector<string> img_tst_path1;
	ifstream img_tst1("f:/最终图片/路径centrist.txt");
	while (img_tst1)
	{
		if (getline(img_tst1, buf))
		{
			img_tst_path1.push_back(buf);
		}
	}
	img_tst1.close();
	vector<string> img_tst_path;
	ifstream img_tst("f:/最终图片/路径new.txt");
	while (img_tst)
	{
		if (getline(img_tst, buf))
		{
			img_tst_path.push_back(buf);
		}
	}
	img_tst.close();*/
    
	Mat img;
	Mat test = frametest;
	char line[512];
	

		IplImage* ntest = NULL;
		//test = imread(img_tst_path1[j].c_str(), 1);
		//ntest = cvLoadImage(img_tst_path[j].c_str(), 1);//读入图像   
		//cout << img_tst_path[j].c_str() << endl;


		resize(test, img, cv::Size(ImgWidht, ImgHeight), 0, 0, INTER_CUBIC);//要搞成同样的大小才可以检测到       
		HOGDescriptor *hog = new HOGDescriptor(cvSize(ImgWidht, ImgHeight), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);  //具体意思见参考文章1,2     
		vector<float>descriptors1;//结果数组     
		hog->compute(img, descriptors1, Size(1, 1), Size(0, 0)); //调用计算函数开始计算 
		cout << "The Detection Result:" << endl;

		Mat SVMtrainMat = Mat::zeros(1, descriptors1.size(), CV_32FC1);
		unsigned long n;
		n = 0;
		for (vector<float>::iterator iter = descriptors1.begin(); iter != descriptors1.end(); iter++)
		{
			SVMtrainMat.at<float>(0, n) = *iter;
			n++;
		}

		int ret = svm.predict(SVMtrainMat);
		//std::sprintf(line, "%s %d\r\n", img_tst_path1[j].c_str(), ret);
		//printf("%s %d\r\n", img_tst_path1[j].c_str(), ret);
		//getchar();
	

		//IplImage* ntest = NULL;
		//test = imread(img_tst_path[j].c_str(), 1);
		//ntest = cvLoadImage(img_tst_path[j].c_str(), 1);//读入图像   
		//cout << img_tst_path[j].c_str() << endl;
		//resize(test, trainImg, cv::Size(ImgWidht, ImgHeight), 0, 0, INTER_CUBIC);
		//vector<float>descriptors;//结果数组     
		//hog->compute(trainImg, descriptors, Size(1, 1), Size(0, 0)); //调用计算函数开始计算 
		//cout << "The Detection Result:" << endl;

		//Mat SVMtrainMat = Mat::zeros(1, descriptors.size(), CV_32FC1);
		//n = 0;
		//for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++)
		//{
		//	SVMtrainMat.at<float>(0, n) = *iter;
		//	n++;
		//}

		//int ret = svm.predict(SVMtrainMat);

		///---------------------------------------------------------------------		
		
		//cvNamedWindow(name, CV_WINDOW_AUTOSIZE);

		//int m_width = 300;
		//int m_height = 300;
		//cvResizeWindow(name, m_width, m_height);
		char *pre = name[ret];
		cout << pre << endl;
		//strcpy(pre, name[ret].c_str());
		//cvPutText(ntest, pre, cvPoint(25, 25), &font, CV_RGB(238, 99, 99));
		//cvShowImage("TEXT", ntest);
		//cvWaitKey(33);
		//Delay(0.004 * 1000);
		//cvDestroyWindow("TEXT");
		//cvReleaseImage(&ntest);
		///---------------------------------------------------------------------

		string re = pre;
	
		
		return re;
}