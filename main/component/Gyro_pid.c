#include stdio.h
#include math.h

#define T   0.01
#define pi  3.1416
#define AIM 0
#define KP	0.1
#define KI	0.1
#define KD	0.0048

//系统状态，包括原变量，一阶量，二阶量。此处为真实状态，区别于下面控制器中计算得到的状态，并且除了角加速度外其他量不能传递给外界
typedef struct {
	double theta;
	double omega;
	double alpha;
} State_t;

State_t s, ps;

//系统初始化，初始角加速度，角速度为零，初始角度为theta0
void init_system(ps p, double theta0)
{
	p.theta=theta0;
	p.omega=0;
	p.alpha=0;
}

//模拟传感器环节，从系统中得到角速度并返回，这里省略测量环节
double sensor(s state)
{
	double feedback;
	feedback=state.omega;
	return feedback;
}

//此处的一组状态量是根据传感器返回的角加速度进行积分微分得到角度和角加速度，并不直接与系统真实状态发生信息传递，模拟真实情况下控制器中记录的状态，并不一定与真实状态相同
void state_calculate(ps p,double feedback)
{
	p.alpha = (feedback-p.omega) * T;
	p.theta += p.omegaT + p.alpha * TT2;
	p.omega = feedback;
}

//控制器解算了当前系统状态后利用pid算法计算修正
double pid_calculate(s state)
{
	double kp=KP,ki=KI,kd=KD;

	double differential;
	static double lastpoint=0,integration;
	
	double fix_angle,fix;
	
	double error;
	
	error=AIM-state.theta;
	
	differential=(error-lastpoint)T;
	integration+=error;
	lastpoint=error;
	
	fix_angle=kperror+kiintegration+kddifferential;
	
	fix=2fix_angleTT-2state.omega-state.alpha;
	
	return fix;
}

//系统根据控制器给出的修正值进行修正，调整状态。此处改变的是真实状态
void adjust_system(ps p, double fix)
{
	p.alpha+=fix;
	p.theta+=p.omega * T+p.alphaTT2;
	p.omega+=p.alphaT;
}

//模拟外力对系统的干扰，在特定时间内对角加速度产生干扰
void disturb_system(ps p,int n)
{
	if(n100&&n200)
		p-alpha+=1000;
}

//p所指向的系统运行，n为运行时间（10ms为单位）
void run_system(ps p,int n)
{
	static s simd_state;					//运行过程中存储在控制器中模拟的系统状态

	double feedback,fix;					//运行过程中传递于各模块之间的中间变量
	
	if(n==1)
		simd_state=p;						//将系统真实的初始状态赋给控制器中的模拟状态，此后两者之间不会直接发生信息的交换
	
	printf(%.2fs  %7.4f  %5.2f,(float)n100,p-theta,simd_state.theta);
	printf(%.2fs  %7.4f,(float)n100,p-alpha);
	printf(%.2fs  %7.4f,(float)n100,p-theta);
	if(n%5==0)
		printf( n);
	else 
		printf(    );
	
	feedback=sensor(p);					//传感器直接从系统真实状态读出角速度作为反馈值
	
	state_calculate(&simd_state,feedback);	//根据传感器的反馈值，系统计算当期系统的状态赋给当前的模拟状态
	
	fix=pid_calculate(simd_state);			//根据解算的模拟状态给出修正值
	
	disturb_system(p,n);					//外力对系统进行干扰
	
	adjust_system(p,fix);					//根据控制器给出的修正值系统的真实状态改变

}


void main()
{
	int n,time=3;							//系统运行时间,time单位是s，n单位是10ms

	s actual_state;							//真实的系统
	
	init_system(&actual_state,-pi12);		//系统初始化
	
	for(n=1;n=time100;n++)				//系统运行
	
		run_system(&actual_state,n);

}



