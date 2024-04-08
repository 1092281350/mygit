#ifndef _ADP_ADRC_H_
#define _ADP_ADRC_H_

#include "stm32f4xx.h"
#include "math.h"

typedef struct
{
	float z1;
	float z2;
	float z2_fil;
	float z3;
	float e;
	float fe;
	float fe_1;
	float beta01;
	float beta02;
	float beta03;

	
	



}ESO_Struct;


//书本P282页
//可有如下说法
//r0 = /h^2
//beta01 beta02 beta03与h存在关系
//beta01 = 1/h,beta02 = 1/(1.6h^1.5),beta03 = 2/(8.6*h^2.2)
//200 1770 26841
//反馈形式2 r=0.5/h^2,c=0.5,h1=5h
//r=20000  c=0.5 h1=0.025\



typedef struct
{
				//Td
	float r0;    //最速控制输出
	float h0;    // 
	float td_v1; //TD的跟踪量与提取到的微分值
	float td_v2;
				 //ESO
	ESO_Struct ESO;
	
	float e1;  //外围误差
	float e2;  //微分误差
	float h;   //积分步长
	float b;   //
	float u;
	float b0;
	float u0;  //反馈组合输出
	
				//反馈形式1 u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)    0<alpha1<1<alpha2 
	float beta_1;
	float beta_2;
	float alpha_1;
	float alpha_2;
	float zeta;
	           //反馈形式2 u0=-fhan(e1,c*e2*e2,r,h1);
	float c;
	float r;
	float h1;
	
	float kp;
	float kd;
	

}ADP_ADRC_Struct;	



typedef struct
{
	float raw;
	float div;
	float raw_last;
	float div_filt;



}Div_S;






void Adrc_Control(ADP_ADRC_Struct *ADRC,float expect,float measure);
void ADP_ADRC_ParaInit(void);
void ADP_ADRC_ParaReset(void);

void Get_div(Div_S *diva,float raw,float filter_coe);








#endif





