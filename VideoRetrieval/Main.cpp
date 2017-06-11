#include "clude.h"
#include "Global.h"
#include "Monitor.h"

int main() {
	// time
	string basedir = "S:\\��Ƶ����\\ObjectDetection\\Video\\";
	time_t start, end;
	start = clock();
	char* file_path = new char[50];
	char* output_path = new char[50];
	for (video_id = 2; video_id <= 2; ++video_id) {
		is_new_video = TRUE;
		string basedir_gt = basedir;//ground_truth��·��
		sprintf(file_path, "S:\\��Ƶ����\\ObjectDetection\\Video\\Test\\test_%d.avi", video_id);
		sprintf(output_path, "S:\\��Ƶ����\\ObjectDetection\\Video\\Display\\output_%d.avi", video_id);
		
		system("md cut");
		remove("statistic.txt");
		svm.load("SVM_DATA.xml");
		CvFont font;
		cvInitFont(&font, CV_FONT_VECTOR0, 1, 1, 0, 1, 8);
		char *scenere;
		scenere = "";
		string scenere_result;
		//screnere();
		Monitor *test_monitor;
		test_monitor = new Monitor();	                  //�½�Monitor�����Monitor��ʵ�ּ�������
		CvCapture* capture = 0;                           //cpture ���ڶ���Ƶ
		IplImage *frame, *frame_copy = 0;
		// speedup
		cvNamedWindow("���Խ��", 1);
		capture = cvCaptureFromFile(file_path);       //������Ƶ ��� 1��¥�������� 2����ͨ������ 3����ͨ������
		cvGrabFrame(capture);
		frame = cvRetrieveFrame(capture);                 //��ȡ��Ƶ�ļ�
		int iW = frame->width;
		int iH = frame->height;
		int stride = frame->widthStep;
		test_monitor->basedir_global = basedir_gt;
		test_monitor->m_FGTrainFrames = 25;
		test_monitor->Init(iW, iH, stride, 2);
		// ��ʼ��Monitor
		// ȡ��Ƶ��ǰ30֡���ڱ�����˹��ģ��������Ч�����ã����Ե�������֡��
		// �����Ƶ
		CvVideoWriter *writer = NULL;
		double input_fps = 25;
		writer = cvCreateVideoWriter(output_path, CV_FOURCC('X', 'V', 'I', 'D'),
			input_fps, cvSize(iW, iH), 1);
		if (capture)
		{
			//cout << test_monitor->m_FrameCount << endl;
			//ѭ����ȡ��Ƶ��ÿһ֡
			for (;;)
			{
				// frame time
				//time_t start_f, end_f;
				//start_f = clock();
				//cout << test_monitor->m_FrameCount << endl;
				// Sleep(15);      

				//�����豸�����ٶȣ���������ٶȽϿ죬��ÿ֡����ͣһС��ʱ��
				if (!cvGrabFrame(capture))
					break;
				frame = cvRetrieveFrame(capture);
				frame_copy = (IplImage *)frame;

				/*if (test_monitor->m_FrameCount % 100 == 1)
				{
				//int *atemp = test_monitor->Crosslinelocation(frame_copy);
				Points TEMP=test_monitor->Crosslinelocation(frame_copy);
				//int x = atemp[0]; x = atemp[1]; x = atemp[2]; x = atemp[3];
				//cout << TEMP.points[0] << "  " << TEMP.points[1] << "  " << TEMP.points[2] << "  " << TEMP.points[3] << endl;
				test_monitor->Crosslineflag = 0;
				test_monitor->Crosslinex1 = TEMP.points[0];
				test_monitor->Crosslinex2 = TEMP.points[1];
				test_monitor->Crossliney1 = TEMP.points[2];
				test_monitor->Crossliney2 = TEMP.points[3];
				Mat frametest(frame, 0);
				scenere_result = screnere_test(frametest);
				scenere = (char*)scenere_result.data();
				}
				*/



				frame = test_monitor->Process(frame_copy, Rule, 1);
				cvPutText(frame, scenere, cvPoint(25, 25), &font, CV_RGB(238, 99, 99));
				cvWriteFrame(writer, frame);
				// speedup
				cvShowImage("���Խ��", frame);



				if (cvWaitKey(1) >= 0)
					break;
				//end_f = clock();
				//printf("Frame Time:%f\n", (double)(end_f - start_f)/CLOCKS_PER_SEC);
			}
			cvReleaseCapture(&capture);
			cvReleaseVideoWriter(&writer);
		}
		// speedup
		cvDestroyWindow("���Խ��");
	}

	string file = basedir + "item_data.txt";
	remove(file.c_str());
	ofstream item_out(file.c_str(), ios::app);
	item_out << "ID\t" << "Cate\t" << "Color1\t" << "Color2\t" << "SFrame\t" << "EFrame\t" << "AbV\t" << "AbTraj\n";
	for (int i = 1; i < item_list.size(); ++i) {
		item_out << i << '\t' << item_list[i].category << '\t';
		int tmp_list[10];
		memcpy(tmp_list, item_list[i].color_list, sizeof(item_list[i].color_list));
		item_list[i].color_t1 = distance(tmp_list, max_element(tmp_list, tmp_list + 10));
		tmp_list[item_list[i].color_t1] = 0;
		item_list[i].color_t2 = distance(tmp_list, max_element(tmp_list, tmp_list + 10));
		item_out << item_list[i].color_t1 << '\t' << item_list[i].color_t2 << '\t'
			<< item_list[i].start_frame << '\t' << item_list[i].end_frame << '\t'
			<< item_list[i].is_abnormal_v << '\t' << item_list[i].is_abnormal_traj << '\n';
	}
	/*
	string file = basedir + "item_data.txt";
	remove(file.c_str());
	ofstream item_out(file.c_str(), ios::app);
	item_out << "ID\t" << "V ID\t" << "ID inV\t" << "Cate\t"
			 << "CL[0]\tCL[1]\tCL[2]\tCL[3]\tCL[4]\tCL[5]\tCL[6]\tCL[7]\tCL[8]\tCL[9]\t"
			 << "Ct1\t" << "Ct2\t" << "Color\t" << "SFrame\t" << "EFrame\t" << "Ab V\t" << "Ab Traj\n";
	for (int i = 1; i < item_list.size(); ++i) {
		item_out << i << '\t' << item_list[i].video_id << '\t' << item_list[i].id << '\t' << item_list[i].category << '\t';
		for (int j = 0; j < 10; ++j) {
			item_out << item_list[i].color_list[j] << '\t';
		}
		int tmp_list[10];
		memcpy(tmp_list, item_list[i].color_list, sizeof(item_list[i].color_list));
		item_list[i].color_t1 = distance(tmp_list, max_element(tmp_list, tmp_list + 10));
		tmp_list[item_list[i].color_t1] = 0;
		item_list[i].color_t2 = distance(tmp_list, max_element(tmp_list, tmp_list + 10));
		item_out << item_list[i].color_t1 << '\t' << item_list[i].color_t2 << '\t' << item_list[i].color
				 << '\t' << item_list[i].start_frame << '\t' << item_list[i].end_frame << '\t'
				 << item_list[i].is_abnormal_v << '\t' << item_list[i].is_abnormal_traj << '\n';
	}
	*/
	item_out.close();
	end = clock();
	printf("Total Time:%f\n", (double)(end - start) / CLOCKS_PER_SEC);
	system("pause");
}