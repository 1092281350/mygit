#include <stdint.h>
#include <math.h>

#define ABS(X)  (((X)>0)?(X):-(X))

typedef struct
{
	/*****安排过度过程*******/
	float x1;    //跟踪微分期状态量
	float x2;    //跟踪微分期状态量微分项
	float r;     //时间尺度
	float h;     //ADRC系统积分时间
	uint16_t N0; //跟踪微分器解决速度超调h0=N*h

	float h0;
	float fh;//最速微分加速度跟踪量

	/*****扩张状态观测器*******/
	/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
	float z1;
	float z2;
	float z3; //根据控制对象输入与输出，提取的扰动信息
	float e;  //系统状态误差
	float y;  //系统输出量
	float fe1;
	float fe2;
	float beta_01;
	float beta_02;
	float beta_03;
	float zeta;   //线性段的区间长度

	/**********系统状态误差反馈率*********/
	float e0; //状态误差积分项
	float e1; //状态偏差
	float e2; //状态量微分项
	float u0; //非线性组合系统输出
	float u;  //带扰动补偿后的输出
	float b0; //扰动补偿


	/*********第一种组合形式*********/
	float k_0; //线性
	float k_1; //非线性组合参数
	float k_2; //u0=beta_1*e1+beta_2*e2+(beta_0*e0);

	/*********第二种组合形式*********/
	float alpha1; //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
	float alpha2; //0<alpha1<1<alpha2
	

	/*********第三种组合形式*********/
	float h1;    //u0=-fhan(e1,e2,r,h1);
	uint16_t N1; //跟踪微分器解决速度超调h0=N*h

	/*********第四种组合形式*********/
	float c;    //u0=-fhan(e1,c*e2*e2,r,h1);
}Fhan_Data;

//const float ADRC_Unit[3][15] =
//{
//	// TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合
//	//  r     h      N                 beta_01   beta_02    beta_03     b0       beta_0  beta_1  beta_2   N1     C    alpha1  alpha2
//	{ 300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50 },
//	{ 300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50 },
//	{ 50000  ,0.005 , 30,              100,      2000,      10000,    5    ,    0.002,   10,        0.001,    5,    5,    0.5,   1.05,   50 },
//};


//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
const float ADRC_Unit[3][15] =
{
	//TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO          扰动补偿           非线性组合
	//  r      h      N                beta_01   beta_02    beta_03      b0         k_0      k_1    k_2       N1     C    alpha1  alpha2   zeta
	{ 75000 ,  0.02,  2,               25,        100,        100,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
	{ 75000 ,  0.02,  2,               100,      1000,       2000,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
	{ 50000 ,  0.005, 30,              100,      2000,      10000,           5,      0.002,   10,     0.001,    5,    5,    0.5,   1.05,     50 },
};

void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2,int base,int base2);

// sign符号函数
int16_t Sign_ADRC(float Input);

int16_t Fsg_ADRC(float x, float d);

//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC);

float Fal_ADRC(float e, float alpha, float zeta);

void ESO_ADRC(Fhan_Data *fhan_Input);

float Constrain_Float(float amt, float low, float high);

void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input);

/*-- ADRC 控制的完整流程 --*/
float ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC);
// ADRC参数初始化
void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2,int base,int base2)
{
	fhan_Input1->r = ADRC_Unit[base][0];
	fhan_Input1->h = ADRC_Unit[base][1];
	fhan_Input1->N0 = (uint16_t)(ADRC_Unit[base][2]);
	fhan_Input1->beta_01 = ADRC_Unit[base][3];
	fhan_Input1->beta_02 = ADRC_Unit[base][4];
	fhan_Input1->beta_03 = ADRC_Unit[base][5];
	fhan_Input1->b0 = ADRC_Unit[base][6];
	fhan_Input1->k_0 = ADRC_Unit[base][7];
	fhan_Input1->k_1 = ADRC_Unit[base][8];
	fhan_Input1->k_2 = ADRC_Unit[base][9];
	fhan_Input1->N1 = (uint16_t)(ADRC_Unit[base][10]);
	fhan_Input1->c = ADRC_Unit[base][11];

	fhan_Input1->alpha1 = ADRC_Unit[base][12];
	fhan_Input1->alpha2 = ADRC_Unit[base][13];
	fhan_Input1->zeta = ADRC_Unit[base][14];

	fhan_Input2->r = ADRC_Unit[base2][0];
	fhan_Input2->h = ADRC_Unit[base2][1];
	fhan_Input2->N0 = (uint16_t)(ADRC_Unit[base2][2]);
	fhan_Input2->beta_01 = ADRC_Unit[base2][3];
	fhan_Input2->beta_02 = ADRC_Unit[base2][4];
	fhan_Input2->beta_03 = ADRC_Unit[base2][5];
	fhan_Input2->b0 = ADRC_Unit[base2][6];
	fhan_Input2->k_0 = ADRC_Unit[base2][7];
	fhan_Input2->k_1 = ADRC_Unit[base2][8];
	fhan_Input2->k_2 = ADRC_Unit[base2][9];
	fhan_Input2->N1 = (uint16_t)(ADRC_Unit[base2][10]);
	fhan_Input2->c = ADRC_Unit[base2][11];

	fhan_Input2->alpha1 = ADRC_Unit[base2][12];
	fhan_Input2->alpha2 = ADRC_Unit[base2][13];
	fhan_Input2->zeta = ADRC_Unit[base2][14];
}


/* ADRC最速跟踪微分器TD，改进的算法fhan */
// sign符号函数
int16_t Sign_ADRC(float Input)
{
	int16_t output = 0;
	if (Input>1E-6) output = 1;
	else if (Input<-1E-6) output = -1;
	else output = 0;
	return output;
}

int16_t Fsg_ADRC(float x, float d)
{
	int16_t output = 0;
	output = (Sign_ADRC(x + d) - Sign_ADRC(x - d)) / 2;
	return output;
}


//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC)//安排ADRC过度过程
{
	float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
	float x1_delta = 0;//ADRC状态跟踪误差项
	x1_delta = fhan_Input->x1 - expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
	fhan_Input->h0 = fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
	d = fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
	a0 = fhan_Input->h0*fhan_Input->x2;//a0=h*x2
	y = x1_delta + a0;//y=x1+a0
	a1 = sqrt(d*(d + 8 * ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
	a2 = a0 + Sign_ADRC(y)*(a1 - d) / 2;//a2=a0+sign(y)*(a1-d)/2;
	a = (a0 + y)*Fsg_ADRC(y, d) + a2*(1 - Fsg_ADRC(y, d));
	fhan_Input->fh = -fhan_Input->r*(a / d)*Fsg_ADRC(a, d)
		- fhan_Input->r*Sign_ADRC(a)*(1 - Fsg_ADRC(a, d));//得到最速微分加速度跟踪量
	fhan_Input->x1 += fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
	fhan_Input->x2 += fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e, float alpha, float zeta)
{
	int16_t s = 0;
	float fal_output = 0;
	s = (Sign_ADRC(e + zeta) - Sign_ADRC(e - zeta)) / 2;
	fal_output = e*s / (powf(zeta, 1 - alpha)) + powf(ABS(e), alpha)*Sign_ADRC(e)*(1 - s);
	return fal_output;
}


/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
	fhan_Input->e = fhan_Input->z1 - fhan_Input->y;//状态误差

	fhan_Input->fe1 = Fal_ADRC(fhan_Input->e, 0.5, fhan_Input->h);//非线性函数，提取跟踪状态与当前状态误差
	fhan_Input->fe2 = Fal_ADRC(fhan_Input->e, 0.25, fhan_Input->h);

	/*************扩展状态量更新**********/
	fhan_Input->z1 += fhan_Input->h * (fhan_Input->z2 - fhan_Input->beta_01*fhan_Input->e);
	fhan_Input->z2 += fhan_Input->h * (fhan_Input->z3 - fhan_Input->beta_02*fhan_Input->fe1 + fhan_Input->b0*fhan_Input->u);
	//ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
	fhan_Input->z3 += fhan_Input->h * (-fhan_Input->beta_03 * fhan_Input->fe2);
}

float Constrain_Float(float amt, float low, float high) {
	return ((amt)<(low) ? (low) : ((amt)>(high) ? (high) : (amt)));
}

void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
	float temp_e2 = 0;
	temp_e2 = Constrain_Float(fhan_Input->e2, -3000, 3000);
	fhan_Input->u0 = fhan_Input->k_1 * Fal_ADRC(fhan_Input->e1, fhan_Input->alpha1, fhan_Input->zeta)
		+ fhan_Input->k_2 * Fal_ADRC(temp_e2, fhan_Input->alpha2, fhan_Input->zeta);

}





/*-- ADRC 控制的完整流程 --*/
float ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC)
{
	/*自抗扰控制器第1步*/
	/********
	**
	**
	**
	**
	**
	********/
	/*****
	安排过度过程，输入为期望给定，
	由TD跟踪微分器得到：
	过度期望信号x1，过度期望微分信号x2
	******/
	Fhan_ADRC(fhan_Input, expect_ADRC);

	/*自抗扰控制器第2步*/
	/********
	*
	*
	****
	*
	*
	********/
	/************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
	fhan_Input->y = feedback_ADRC;
	/*****
	扩张状态观测器，得到反馈信号的扩张状态：
	1、状态信号z1；
	2、状态速度信号z2；
	3、状态加速度信号z3。
	其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
	经过非线性函数映射，乘以beta系数后，
	组合得到未加入状态加速度估计扰动补偿的原始控制量u
	*********/
	ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
						 /*自抗扰控制器第3步*/
						 /********
						 **
						 **
						 **
						 **
						 **
						 ********/
						 /********状态误差反馈率***/
	fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h;//状态积分项
	fhan_Input->e1 = fhan_Input->x1 - fhan_Input->z1;//状态偏差项
	fhan_Input->e2 = fhan_Input->x2 - fhan_Input->z2;//状态微分项，
	
	/********线性组合*******/
	/*
	fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
	+fhan_Input->beta_1*fhan_Input->e1
	+fhan_Input->beta_2*fhan_Input->e2;
	*/
	Nolinear_Conbination_ADRC(fhan_Input);
	/**********扰动补偿*******/
	//fhan_Input->u=fhan_Input->u0
	//             -fhan_Input->z3/fhan_Input->b0;
	//由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
	fhan_Input->u = Constrain_Float(fhan_Input->u0, -200, 200);
    return fhan_Input->u;
}
