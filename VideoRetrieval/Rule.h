#ifndef rule_H
#define rule_H

typedef struct
{

	int Rule_foregroundXsize;                 // ǰ�������СX
	int Rule_foregroundYsize;				   // ǰ�������СY
	int areashape[5];						// ������״���  0:�����Σ�1�������

	double PointX[5][20];	            //��¼ѡ��������
	double PointY[5][20];              // ��¼ѡ�������꣬�������򶥵�˳ʱ���λ��

	int invasiontime;                  // ���ֵ������е�����ʱ��

	int invasion_flag;                // ��ʶ������1�������������ּ�⣬0����û��
	int linecross_flag;               //1������߼��
	int wander_flag;                  //1�����ǻ�
	int remove_flag;                  //1��������
	int suddenappear_flag;            //1����ͻȻ����
	int suddenAccelerate_flag;        //1����ͻȻ����    
	int route_flag;                   //1����·�����
	int stay_flag;                    //1�����������
	int dropbag_flag;                 //1�����������

	//double LinePointX[5][2];	          //��¼���߼��ֱ�ߵĺ����꣬���֧��5����
	//double LinePointY[5][2];             //��¼���߼��ֱ�ߵ������꣬���֧��5����

	double LineStartX[5];
	double LineStartY[5];
	double LineEndX[5];
	double LineEndY[5];
	int LineDirection[5];                //��¼���ߵķ���1Ϊ��1����2����2Ϊ��2����1��

	int  NumOfArea;            //���õ�������Ŀ
	int  NumOfLine;            //���õİ�����Ŀ
	int  NumPointArea[5];      //ÿ�������ڵĶ�����


}rule_son;



typedef struct
{
	rule_son Rule_invasion;                 //����
	rule_son Rule_linecross;                //����
	rule_son Rule_wander;                   //�ǻ�
	rule_son Rule_remove;                   //����
	rule_son Rule_suddenappear;             //ͻȻ����
	rule_son Rule_suddenAccelerate;         //ͻȻ����
	rule_son Rule_route;	                  //·��
	rule_son Rule_stay;                     //����
	rule_son Rule_dropbag;                  //������������

}rule;                                        // ��һ·����ṹ��


typedef struct
{
	int points[10];
}Points;

enum catagory{vehicle,pedestrian};

#endif