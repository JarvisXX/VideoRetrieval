#ifndef STC
#define STC
//#include "CarPedDetector.h"
#define NUMOFWEAK 400
#define PEDNUMOFWEAK 600
#define CLASSVALVE 5.0
class tree_node    //树分类节点
{

public:
	tree_node(void);
	~tree_node(void);
public:
	float left_constrain; //节点对应的左阈值
    float right_constrain;   //节点对应的右阈值
	int dim;     //一个节点对应的维数属性
	int max_split;      //节点的最大分叉数？
	bool hasparent;  //是否包含父节点
	tree_node*parent;     //节点的父亲节点？      
};
class CStrongTreeClassifier   //强分类器
{
public:
	CStrongTreeClassifier(void);
	~CStrongTreeClassifier(void);
private:
	// 一个强分类器包含的弱分类器个数
	int Numofweakclassifier;

public:
	//初始化分类器
	void InitialStrongClassifier(int numofweakclassifer,const char* variablefile);
	void ReadVariableWeak(fstream*files, tree_node* treeweak,int no);
    double classvalue;
	float* GWeights;
	// 弱分类器
	tree_node* WeakClassifiers[600];
public:
	void WriteVariableWeak(ofstream* files, tree_node* treeweak,int no);
	void TestReadWrite(const char*variablefile);
	int Classify(float*feature);  // 返回分类为目标的可能性
	int cal_singleweak(tree_node* weaktree,float*feature);
	double Classify(float* feature, int possiblity);
};

#endif