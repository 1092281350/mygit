#include "Adp_Adrc.h"

#include "filter.h"

/*  ������ */
extern uint16_t THR_Out;
extern float Ultra_Hnow;

/*  �ṹ���� */
ADP_ADRC_Struct ADRC_ROL_RATE;
ADP_ADRC_Struct ADRC_PIT_RATE;
ADP_ADRC_Struct ADRC_YAW_RATE;

Div_S ESO_Z3;

/*  ������ */
float e1_fal,e2_fal;


const float ADRC_Unit[3][17]=
{
  /*TD����΢����               ����״̬�۲���ESO         �Ŷ�����                             ���������*/
  /*  r0     h0       h    beta_01 beta_02 beta_03    b0    beta_0  beta_1   beta_2     h1      c      r    alpha1  alpha2   zeta    b*/
  {1000000 ,0.005 , 0.005,  200,  1770,    26841,    150,   0.85,   1.5,     0.0003,   0.025,   0.5,  20000,  0.9,   1.2,    0.03,  0.5},
  {1000000 ,0.005 , 0.005,  300,  10000,   100000,   0.5,   0.85,   1.5,     0.0003,   0.025,   0.5,  20000,  0.9,   1.2,    0.03,  0.1},
  {300000  ,0.005 , 0.005,  300,  4000,    10000,    100,   0.2,    1.2,     0.0010,   0.025,   0.5,  20000,  0.8,   1.5,    0.03,  0.05},
};


void ADP_ADRC_ParaInit(void)
{
	ADRC_ROL_RATE.r0 = 20000;
//	ADRC_ROL_RATE.h0 = 0.005;
	ADRC_ROL_RATE.h0 = 0.008;
	ADRC_ROL_RATE.h = 0.005;

	ADRC_ROL_RATE.td_v1 = 0.0f;
	ADRC_ROL_RATE.td_v2 = 0.0f;
	
	//ESO
//	ADRC_ROL_RATE.ESO.beta01 = ADRC_Unit[0][3];
//	ADRC_ROL_RATE.ESO.beta02 = ADRC_Unit[0][4];
//	ADRC_ROL_RATE.ESO.beta03 = ADRC_Unit[0][5];
									 //ʦ�ֵĲ���
//	ADRC_ROL_RATE.ESO.beta01 = 300;
//	ADRC_ROL_RATE.ESO.beta02 = 7000;
//	ADRC_ROL_RATE.ESO.beta03 = 1000;
	
	float w0 = 30;
	ADRC_ROL_RATE.ESO.beta01 = 3*w0;
	ADRC_ROL_RATE.ESO.beta02 = 3*w0*w0;
	ADRC_ROL_RATE.ESO.beta02 = 1200;
//	ADRC_ROL_RATE.ESO.beta03 = w0*w0*w0;
	ADRC_ROL_RATE.ESO.beta03 = 1000;
	ADRC_ROL_RATE.ESO.beta03 = 700;
	
	
	//140 6000 200000 
	//b = 60
	ADRC_ROL_RATE.ESO.z1 = 0.0f;
	ADRC_ROL_RATE.ESO.z2 = 0.0f;
	ADRC_ROL_RATE.ESO.z3 = 0.0f;
	
    ADRC_ROL_RATE.u = 0;
	
	//����
	ADRC_ROL_RATE.beta_1 = ADRC_Unit[0][8];
	ADRC_ROL_RATE.beta_2 = ADRC_Unit[0][9];
	ADRC_ROL_RATE.alpha_1 = ADRC_Unit[0][13];
	ADRC_ROL_RATE.alpha_2 = ADRC_Unit[0][14];
	ADRC_ROL_RATE.zeta = ADRC_Unit[0][15];
	
		//����	
	ADRC_ROL_RATE.kp = 1.3f;
	ADRC_ROL_RATE.kd = 0.02f;
	
	//�Ŷ�����	
	ADRC_ROL_RATE.b = 10.0f;
	
	//�ȹ۲�����Ӱ�� b w0 kp kd
	//b��0.1��1000����    //b��С����z�Ĺ��ƴ��ͺ���ǰ�������г�����bȡ1����ʱ�Ὣ ��b�ܴ�ʱz�Ĺ��ƻ��нϴ��ƫ�����z2Ӱ��ܴ�
	//w0��1��1000���� //wo��С����һ��ʼz�Ĺ��ƻ�ǳ��ͺ���ƽ������������ᷢ����λ�ͺ���ٵ���z3�Ĺ��ƻ��󣬲����ǳ�Ѹ��  	
	
	ADRC_PIT_RATE.r0 = 20000;
	ADRC_PIT_RATE.h0 = 0.008;
	ADRC_PIT_RATE.h = 0.005;

	ADRC_PIT_RATE.td_v1 = 0.0f;
	ADRC_PIT_RATE.td_v2 = 0.0f;
	
	ADRC_PIT_RATE.ESO.beta01 = 3*w0;
	ADRC_PIT_RATE.ESO.beta02 = 3*w0*w0;
//	ADRC_PIT_RATE.ESO.beta03 = w0*w0*w0;
	ADRC_PIT_RATE.ESO.beta02 = 1200;
	ADRC_PIT_RATE.ESO.beta03 = 1000;
	ADRC_PIT_RATE.ESO.beta03 = 700;
		
	ADRC_PIT_RATE.ESO.z1 = 0.0f;
	ADRC_PIT_RATE.ESO.z2 = 0.0f;
	ADRC_PIT_RATE.ESO.z3 = 0.0f;                                   
	
	//����
//	ADRC_PIT_RATE.kp = 1.0f;
//	ADRC_PIT_RATE.kd = 0.01f;   

	ADRC_PIT_RATE.kp = 1.3f;
	ADRC_PIT_RATE.kd = 0.02f; 
	
	//�Ŷ�����	
	ADRC_PIT_RATE.b = 10.0f;
   //kp 1 kd 0.01 w=15 b = 10 ������ ��u0������u
   //kp 1 kd 0.01 w=15 b = 10 ������ beta3ȡ1000 ��u 
   //kp 1 kd 0.01 w=25 b = 15 ������ beta3ȡ1000 ��u Ч���úܶ�
   //kp 1 kd 0.01 w=35 b = 15 ������ beta3ȡ1000 ��u  Ч������һ���ֺ���һ�� ����һ��
   //�ټӴ���� Ȼ��e2��΢�˲�����΢�ֿ��� ���߿��ǽ�z2�˲� 
  
}


float Sign_func(float u)
{
	
	if(u>1e-3f)
		u = 1.0f;
	else if(u<-1e-3f)
		u = -1.0f;
	else 
		u = 0.0f;
	return u;
	
}

int16_t Fsg_func(float x,float d)
{
	
	int16_t output=0;
	output=(Sign_func(x+d)-Sign_func(x-d))/2;
	return output;

}

float Sat_func(const float x,const float sigma)
{
	
	float y;
	y=Fsg_func(x,sigma)*x/sigma+(1-Fsg_func(x,sigma))*Sign_func(x);
	return y;

}

float Fal_func(float e,float alpha,float zeta)
{
	
	float s=0;
    float fal_output=0.0f;
    s=(Sign_func(e+zeta)-Sign_func(e-zeta))/2.0f;
    fal_output=e*s/(powf(zeta,1.0f-alpha))+powf(fabs(e),alpha)*Sign_func(e)*(1.0f-s);
    return fal_output;
	
}


//ԭ������Ӧ��д���ˣ� �ڶ����ǣ������Ǹ�
float Fsun_track(float v1, float v2, float r0, float h0)
{
	
	float u_out;
	float y = v1 + h0 * v2;
	float y_abs = fabs(y);
	float d = h0 * h0 * r0;
	float kd = 0.5f * (1+sqrt(1+8*y_abs/d));
	uint16_t fix_kd = kd;
	float k = Sign_func(kd - fix_kd) + fix_kd;
	if(y_abs<=d)
	{
		u_out = - r0 * Sat_func(v2+y/h0,h0*r0);
	}
	else
	{
		//u_out =  r0 * Sat_func((1-0.5f*k)*Sign_func(v1)-(v1+k*h0*v2)/((k-1)*d),1);
		u_out = r0 * Sat_func((1-0.5f*k)*Sign_func(y)-(v1+k*h0*v2)/((k-1)*d),1);
	}
	return u_out;
	
}


//TD�˲������Ź��ɹ���
void ADRC_TD(ADP_ADRC_Struct *fsun, float expect)
{
	
	float fv = Fsun_track(fsun->td_v1 - expect, fsun->td_v2, fsun->r0, fsun->h0);	
	fsun->td_v1 += fsun->h * fsun->td_v2;
	fsun->td_v2 += fsun->h * fv; 
 	
}



//����
void Error_Feedback(ADP_ADRC_Struct *ADRC,float kp,float kd)
{
	
	ADRC->u0 = kp * ADRC->e1 + kd * ADRC->e2;

}

	
//ESO
void ESO_3order_(ADP_ADRC_Struct *ADRC,float measure)
{

	ADRC->ESO.e = ADRC->ESO.z1 - measure; //����״̬�۲�����y�ǲ���ֵ

	ADRC->ESO.z1 += ADRC->h*(ADRC->ESO.z2 - ADRC->ESO.beta01 * ADRC->ESO.e);
	ADRC->ESO.z2 += ADRC->h*(ADRC->ESO.z3 - ADRC->ESO.beta02 * ADRC->ESO.e + ADRC->b * ADRC->u);
	ADRC->ESO.z3 += ADRC->h*(- ADRC->ESO.beta03 * ADRC->ESO.e);	
	
}

//void ESO_3order_(ADP_ADRC_Struct *ADRC,float measure)
//{

//	ADRC->ESO.e = ADRC->ESO.z1 - measure; //����״̬�۲�����y�ǲ���ֵ

//	ADRC->ESO.z1 += ADRC->h*(ADRC->ESO.z2 - ADRC->ESO.beta01 * ADRC->ESO.e);
//	ADRC->ESO.z2_fil += ADRC->h*(ADRC->ESO.z3 - ADRC->ESO.beta02 * ADRC->ESO.e + ADRC->b * ADRC->u);
//	RC_Filter_(0.98,ADRC->ESO.z2_fil,&ADRC->ESO.z2);
//	
//	ADRC->ESO.z3 += ADRC->h*(- ADRC->ESO.beta03 * ADRC->ESO.e);	
//	
//}


void Get_div(Div_S *diva,float raw,float filter_coe)
{
	diva->raw = raw;
	
	diva->div = (diva->raw - diva->raw_last)*200.0f;
	RC_Filter_(filter_coe,diva->div,&diva->div_filt); 
		
	diva->raw_last = diva->raw;

}


void Adrc_Control(ADP_ADRC_Struct *ADRC,float expect,float measure)
{

	//����Ҫ��΢�˲�
	ADRC_TD(ADRC,expect);//Td���Ź��ɹ���

	ESO_3order_(ADRC,measure);//ESO�۲�״̬���Ŷ�
	
	//����һ�ְ취 ����ֱ�Ӹ�����
	if(Ultra_Hnow<21.f)
	{
		ADRC->ESO.z3 = 0;
	}
	
	//Get_div(&ESO_Z3,measure,0.95);

//	ADRC->e1 = ADRC->td_v1 - ADRC->ESO.z1;  //�������
	ADRC->e1 = expect - ADRC->ESO.z1;  //�������
	ADRC->e2 = ADRC->td_v2 - ADRC->ESO.z2;	

	Error_Feedback(ADRC,ADRC->kp,ADRC->kd);  //����

	ADRC->u = ADRC->u0 - (ADRC->ESO.z3/ADRC->b);  //�Ŷ�������������� 
	
}

//������λ����
void ADP_ADRC_ParaReset(void)
{

	ADRC_ROL_RATE.td_v1 = 0.0f;
	ADRC_ROL_RATE.td_v2 = 0.0f;
	
	ADRC_ROL_RATE.ESO.z1 = 0.0f;
	ADRC_ROL_RATE.ESO.z2 = 0.0f;
	ADRC_ROL_RATE.ESO.z2_fil = 0.0f;
	ADRC_ROL_RATE.ESO.z3 = 0.0f;
	
    ADRC_ROL_RATE.u = 0;
	ADRC_ROL_RATE.u0 = 0;
	ADRC_ROL_RATE.e1 = 0;
	ADRC_ROL_RATE.e2 = 0;
    ADRC_ROL_RATE.ESO.e = 0;
	
	ADRC_PIT_RATE.td_v1 = 0.0f;
	ADRC_PIT_RATE.td_v2 = 0.0f;
	
	ADRC_PIT_RATE.ESO.z1 = 0.0f;
	ADRC_PIT_RATE.ESO.z2 = 0.0f;
	ADRC_PIT_RATE.ESO.z2_fil = 0.0f;
	ADRC_PIT_RATE.ESO.z3 = 0.0f;
	
    ADRC_PIT_RATE.u = 0;
	ADRC_PIT_RATE.u0 = 0;
	ADRC_PIT_RATE.e1 = 0;
	ADRC_PIT_RATE.e2 = 0;
    ADRC_PIT_RATE.ESO.e = 0;

	
}








