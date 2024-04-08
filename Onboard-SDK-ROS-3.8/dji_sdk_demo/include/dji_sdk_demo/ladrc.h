#ifndef _LADRC_H
#define _LADRC_H
/**
   *@Brief  ����ΪLADRCϵͳ����
   *@WangShun  2022-07-03  ע��
   */
typedef struct LADRC
{
    float v1,v2;         //�������ֵ
    float r;             //�ٶ�����
    float h;             //���ֲ���
    float z1,z2,z3;      //�۲������
    float w0,wc,b0,u;    //�۲������� ���������� ϵͳ���� ���������
}LADRC_NUM;

/*
	wu = 2*3.1415/Pu;
    ku = 4*h/3.1415*a;

	wc = 2.3997wu - 0.4731;
	w0 = 0.7332wu + 3.5070;
	b0 = 3.6105wu + 4.8823;
*/
typedef struct Auto_Tuning 
{
	float Pu; //�̵�ʵ���������
	float a;  //�̵�ʵ�������ֵ
	float h;  //ָ�������ֵ
	float Wu; //ϵͳ�ٽ�Ƶ��
	float Kp; //ϵͳ�ٽ��ֵ
}AuTu;

/**
   *@Brief  ����Ϊ��Ҫ�����Ĳ���
   *@WangShun  2022-07-03  ע��
   */
extern LADRC_NUM  Yaw_Sysparam;
extern LADRC_NUM  Depth_Sysparam; 
extern LADRC_NUM  Pitch_Sysparam; 
/**
   *@Brief  ����ΪLADRC��غ���
   *@WangShun  2022-07-03  ע��
   */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1,int i);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,double Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,double FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1);
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,double* Expect,double* RealTimeOut);

LADRC_NUM   Yaw_Sysparam;    //ϵͳ��Ҫ�����Ĳ��� 
LADRC_NUM   Depth_Sysparam; 
LADRC_NUM   Pitch_Sysparam; 

/**
   *@Brief default ������
   *@Brief ���ݾ��� ts��0.2~0.3֮�� Wo��Wcѡ�����С��������b0
   *@Date@WangShun 2022-05-28 2022-07-03����
---------------------------------------------------
		      LADRC default������									
---------------------------------------------------
---------------------------------------------------
	ts	|	h	|	r	|   wc   |   w0  |	b0
---------------------------------------------------
	0.1	|	h	|	r	|  100   |  400  |	b0
---------------------------------------------------
   0.157|	h	|	r	|   64   |  224~255  |	b0
---------------------------------------------------
   0.158|	h	|	r	|   63   |  253  |	b0
---------------------------------------------------
   0.159|	h	|	r	|   63   |  252  |	b0
---------------------------------------------------
	0.16|	h	|	r	|   63   |  250  |	b0
---------------------------------------------------
	0.17|	h	|	r	|   59   |  235  |	b0
---------------------------------------------------
	0.18|	h	|	r	|   56   |  222  |	b0
---------------------------------------------------
	0.2	|	h	|	r	|   50   |  200  |	b0
---------------------------------------------------
	0.21|	h	|	r	|   48   |  190  |	b0
---------------------------------------------------
	0.22|	h	|	r	|   45   |  182  |	b0
---------------------------------------------------
	0.23|	h	|	r	|   43   |  174  |	b0
---------------------------------------------------
	0.24|	h	|	r	|   42   |  167  |	b0
---------------------------------------------------
	0.25|	h	|	r	|   40   |  160  |	b0
---------------------------------------------------
	0.26|	h	|	r	|   38   |  154  |	b0
---------------------------------------------------
	0.27|	h	|	r	|   37   |  148 |	b0
---------------------------------------------------
	0.28|	h	|	r	|   36   |  144  |	b0
---------------------------------------------------
	0.29|	h	|	r	|   34   |  138  |	b0
---------------------------------------------------
	0.3	|	h	|	r	|   33   |  133  |	b0
---------------------------------------------------
	0.4	|	h	|	r	|   25   |  100  |	b0
---------------------------------------------------
	0.5	|	h	|	r	|   20   |   80  |	b0
---------------------------------------------------
---------------------------------------------------
*/
 
 /**
  * ����˵�� LADRC��ʼ�ο�ֵ
  * 		WangShun��2022-07-03����
  */	
const float LADRC_Unit[6][5]=
{
	{0.005,20,100,400,0.5},
	{0.001,20,33,133,8},
	{0.005,100,20,80,0.5},
	{0.005,100,14,57,0.5},
	{0.005,100,50,10,1},
  {0.02,20000,30,4,8}
};
/**
  * ����˵����LADRC��ʼ��
  * 		WangShun��2022-07-03����
  */	
void LADRC_Init(LADRC_NUM *LADRC_TYPE1,int i)
{
	LADRC_TYPE1->h= LADRC_Unit[i][0]; //��ʱʱ�估ʱ�䲽��
    LADRC_TYPE1->r =LADRC_Unit[i][1]; //�����ٶȲ���
    LADRC_TYPE1->wc=LADRC_Unit[i][2]; //�۲�������
    LADRC_TYPE1->w0=LADRC_Unit[i][3]; //״̬�����ʴ���
	LADRC_TYPE1->b0=LADRC_Unit[i][4]; //ϵͳ����
}
/**
  * ����˵����LADRCȱʡ
  * 		WangShun��2022-07-03����
  */
void LADRC_REST(LADRC_NUM *LADRC_TYPE1)
{
	LADRC_TYPE1->z1= 0; //��ʱʱ�估ʱ�䲽��
    LADRC_TYPE1->z2 =0; //�����ٶȲ���
    LADRC_TYPE1->z3=0; //�۲�������

}

//替换后的TD
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
//替换后的TD
int16_t Fsg_func(float x,float d)
{
	
	int16_t output=0;
	output=(Sign_func(x+d)-Sign_func(x-d))/2;
	return output;

}
//替换后的TD
float Sat_func(const float x,const float sigma)
{
	
	float y;
	y=Fsg_func(x,sigma)*x/sigma+(1-Fsg_func(x,sigma))*Sign_func(x);
	return y;

}
//替换后的TD
float Fal_func(float e,float alpha,float zeta)
{
	
	float s=0;
    float fal_output=0.0f;
    s=(Sign_func(e+zeta)-Sign_func(e-zeta))/2.0f;
    fal_output=e*s/(powf(zeta,1.0f-alpha))+powf(fabs(e),alpha)*Sign_func(e)*(1.0f-s);
    return fal_output;
	
}
//替换后的TD
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

/**
  * ��������void ADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect)
  * ����˵����LADRC����΢�ֲ���
  * @param[in]	��ڲ���������ֵExpect(v0)���ֵv1,v2
  * @par �޸���־
  * 		WangShun��2022-05-28����
  */
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,double Expect)
{
    // float fh= -LADRC_TYPE1->r*LADRC_TYPE1->r*(LADRC_TYPE1->v1-Expect)-2*LADRC_TYPE1->r*LADRC_TYPE1->v2;
    // LADRC_TYPE1->v1+=LADRC_TYPE1->v2*LADRC_TYPE1->h;
    // LADRC_TYPE1->v2+=fh*LADRC_TYPE1->h;

    //替换后的TD
    	float fv = Fsun_track(LADRC_TYPE1->v1-Expect, LADRC_TYPE1->v2, LADRC_TYPE1->r, LADRC_TYPE1->h);	
	LADRC_TYPE1->v1+= LADRC_TYPE1->h * LADRC_TYPE1->v2;
	LADRC_TYPE1->v2 +=LADRC_TYPE1->h * fv; 
}
/**
  * ��������LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack)
  * ����˵����LADRC����״̬�۲���
  * @param[in]
  * @par �޸���־
  * 		WangShun��2022-07-03����
  */
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,double FeedBack)
{
    float Beita_01=3*LADRC_TYPE1->w0;
    float Beita_02=3*LADRC_TYPE1->w0*LADRC_TYPE1->w0;
    float Beita_03=LADRC_TYPE1->w0*LADRC_TYPE1->w0*LADRC_TYPE1->w0;

//线性
    float e= LADRC_TYPE1->z1-FeedBack;
    LADRC_TYPE1->z1+= (LADRC_TYPE1->z2 - Beita_01*e)*LADRC_TYPE1->h;
    LADRC_TYPE1->z2+= (LADRC_TYPE1->z3 - Beita_02*e + LADRC_TYPE1->b0*LADRC_TYPE1->u)*LADRC_TYPE1->h;
    LADRC_TYPE1->z3+=-Beita_03*e*LADRC_TYPE1->h;
    //非线性
    // float e= LADRC_TYPE1->z1-FeedBack;
    // float fe=Fal_func(e,0.5,0.005);
    // float fe1=Fal_func(e,0.25,0.005);
    // LADRC_TYPE1->z1+= (LADRC_TYPE1->z2 - Beita_01*e)*LADRC_TYPE1->h;
    // LADRC_TYPE1->z2+= (LADRC_TYPE1->z3 - Beita_02*fe + LADRC_TYPE1->b0*LADRC_TYPE1->u)*LADRC_TYPE1->h;
    // LADRC_TYPE1->z3+=-Beita_03*fe1*LADRC_TYPE1->h;
}
/**
   *@Brief  LADRC_LSEF
   *@Date   ���Կ�����
			WangShun��2022-07-03����
   */
void LADRC_LF(LADRC_NUM *LADRC_TYPE1)
{
    float Kp=LADRC_TYPE1->wc*LADRC_TYPE1->wc;
    float Kd=2*LADRC_TYPE1->wc;
	/**
       *@Brief  ���Կ�����������kd = 2wc
       *@Before Kd=3*LADRC_TYPE1->wc;
       *@Now    Kd=2*LADRC_TYPE1->wc;
       *@WangShun  2022-04-27  ע��
       */

    float e1=LADRC_TYPE1->v1-LADRC_TYPE1->z1;
    float e2=LADRC_TYPE1->v2-LADRC_TYPE1->z2;
    float u0=Kp*e1+Kd*e2;
    LADRC_TYPE1->u=(u0-LADRC_TYPE1->z3)/LADRC_TYPE1->b0;
	if(LADRC_TYPE1->u>2000)
		LADRC_TYPE1->u=2000;
	else if(LADRC_TYPE1->u<-2000)
		LADRC_TYPE1->u=-2000;
}
/**
  * LADRC���ƺ��� .
  * ������������ѭ���м���
  * @par ����
  * @par �޸���־
  * @WangShun  2022-07-03  ע��
  */
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,double* Expect,double* RealTimeOut)
{
	double  Expect_Value = *Expect;
	double  Measure = *RealTimeOut;
    LADRC_TD(LADRC_TYPE1,Expect_Value);
    LADRC_ESO(LADRC_TYPE1,Measure); 
    LADRC_LF(LADRC_TYPE1);
}


#endif
