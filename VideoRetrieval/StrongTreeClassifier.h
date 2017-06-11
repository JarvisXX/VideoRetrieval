#ifndef STC
#define STC
//#include "CarPedDetector.h"
#define NUMOFWEAK 400
#define PEDNUMOFWEAK 600
#define CLASSVALVE 5.0
class tree_node    //������ڵ�
{

public:
	tree_node(void);
	~tree_node(void);
public:
	float left_constrain; //�ڵ��Ӧ������ֵ
    float right_constrain;   //�ڵ��Ӧ������ֵ
	int dim;     //һ���ڵ��Ӧ��ά������
	int max_split;      //�ڵ�����ֲ�����
	bool hasparent;  //�Ƿ�������ڵ�
	tree_node*parent;     //�ڵ�ĸ��׽ڵ㣿      
};
class CStrongTreeClassifier   //ǿ������
{
public:
	CStrongTreeClassifier(void);
	~CStrongTreeClassifier(void);
private:
	// һ��ǿ������������������������
	int Numofweakclassifier;

public:
	//��ʼ��������
	void InitialStrongClassifier(int numofweakclassifer,const char* variablefile);
	void ReadVariableWeak(fstream*files, tree_node* treeweak,int no);
    double classvalue;
	float* GWeights;
	// ��������
	tree_node* WeakClassifiers[600];
public:
	void WriteVariableWeak(ofstream* files, tree_node* treeweak,int no);
	void TestReadWrite(const char*variablefile);
	int Classify(float*feature);  // ���ط���ΪĿ��Ŀ�����
	int cal_singleweak(tree_node* weaktree,float*feature);
	double Classify(float* feature, int possiblity);
};

#endif