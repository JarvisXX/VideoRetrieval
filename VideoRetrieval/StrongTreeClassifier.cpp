#include "clude.h"
#include "StrongTreeClassifier.h"

tree_node::tree_node(void)
{
	left_constrain=-1; //节点对应的左阈值
	right_constrain=-1;   //节点对应的右阈值
	dim=-1;     //一个节点对应的维数属性
	max_split=-1;      //节点的最大分叉数？
	hasparent=false;  //是否包含父节点
	parent=NULL;     //节点的父亲节点？
}
tree_node::~tree_node(void)
{
 	if(hasparent)
 		delete parent;
}

CStrongTreeClassifier::CStrongTreeClassifier(void)
: Numofweakclassifier(0)
, GWeights(NULL)
,classvalue(0)
{
	
}

CStrongTreeClassifier::~CStrongTreeClassifier(void)
{

	   if(GWeights!=NULL)
	   {
		   delete GWeights;
	   }
	   if(Numofweakclassifier!=0)
	   {
		   for(int i=0;i<Numofweakclassifier;i++)
			   delete WeakClassifiers[i];
	   }		
}

void CStrongTreeClassifier::InitialStrongClassifier(int numofweakclassifer, const char* variablefile)
{
	Numofweakclassifier=numofweakclassifer;
	for(int i=0;i<Numofweakclassifier;i++)
	{
		WeakClassifiers[i]=new tree_node;
	}
	GWeights=new float[numofweakclassifer];
    fstream files;
	files.open(variablefile,ios::in);     //读取各个弱分类器的参数
	for(int i=0;i<numofweakclassifer;i++)
	{
            ReadVariableWeak(&files,WeakClassifiers[i],i);
	}
	files.close();
}

void CStrongTreeClassifier::ReadVariableWeak(fstream*files, tree_node* treeweak,int no)
{
 	(*files)>>treeweak->left_constrain;
 	(*files)>>treeweak->right_constrain;
	(*files)>>treeweak->dim;
	treeweak->dim=treeweak->dim-1;
	(*files)>>treeweak->max_split;
	int tempt;
	(*files)>>tempt;
	if(tempt==0)
	{
		treeweak->hasparent=false;
		(*files)>>GWeights[no];
	}
	else if(tempt==1)
	{
		treeweak->parent=new tree_node;
		treeweak->hasparent=true;
		ReadVariableWeak(files,treeweak->parent,no);
	}
}

void CStrongTreeClassifier::WriteVariableWeak(ofstream* files, tree_node* treeweak,int no)
{

	(*files)<<treeweak->left_constrain<<"\n";
	(*files)<<treeweak->right_constrain<<"\n";
	(*files)<<treeweak->dim<<"\n";
	(*files)<<treeweak->max_split<<"\n";
	if(treeweak->hasparent)
	{
         (*files)<<1<<"\n";
         WriteVariableWeak(files, treeweak->parent,no);
	}
	else
	{
		(*files)<<0<<"\n";
		(*files)<<GWeights[no]<<"\n";
	}

}

void CStrongTreeClassifier::TestReadWrite(const char*variablefile)
{
	ofstream files1;
	files1.open(variablefile,ios::trunc);     //读取各个弱分类器的参数
	for(int i=0;i<Numofweakclassifier;i++)
	{
		WriteVariableWeak(&files1,WeakClassifiers[i],i);
	}


}
// 返回分类为目标的可能性
int CStrongTreeClassifier::Classify(float*feature)
{
	int outvalue=0;
	float sumvalue=0;
	for(int i=0;i<Numofweakclassifier;i++)
	{
		sumvalue+=GWeights[i]*cal_singleweak(WeakClassifiers[i],feature);
	}
	return sumvalue;
	if(sumvalue>CLASSVALVE)
	{
		return 1;
	}
	else
	{
	   return 0;
	}
}
int CStrongTreeClassifier::cal_singleweak(tree_node* weaktree,float*feature)
{
    int value_out=1;
	if(weaktree->hasparent)
	{
		value_out*=cal_singleweak(weaktree->parent,feature);
	}
    if(weaktree->right_constrain!=-1)
	{
		if(feature[weaktree->dim]<weaktree->right_constrain)
		{
            value_out*=1;
		}
		else
		{
			value_out=0;
		}
	}
	if(weaktree->left_constrain!=-1)
	{
		if(feature[weaktree->dim]>weaktree->left_constrain)
		{
			value_out*=1;
		}
		else
		{
			value_out=0;
		}
	}
	return value_out;
}

double CStrongTreeClassifier::Classify(float* feature, int possiblity)
{
	int outvalue=0;
	float sumvalue=0;
	for(int i=0;i<Numofweakclassifier;i++)
	{
		sumvalue+=GWeights[i]*cal_singleweak(WeakClassifiers[i],feature);
	}
	if(sumvalue>2*CLASSVALVE)
	{
		sumvalue=2*CLASSVALVE;
	}
	double e_value=sumvalue-CLASSVALVE;

	double temptvalue=exp(e_value);

     return temptvalue/(exp(CLASSVALVE));
}
