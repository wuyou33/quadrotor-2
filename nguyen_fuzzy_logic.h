#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
/*
Fuzzy logic
Author: nguyen
Date : 13/5/2016

Fuzzy process:
Step 1: Define MF (FuzzySet - Membershipfunction)
Step 2: Define Fuzzy Rule
Step 3: Fuzzification: real value x => degree(strength) of MF input and degree(strength)) MF output
Step 4: Apply Rule (apply Fuzzy Rule) 
				Fuzzy Rule + (degree MF input_1 & degree MF input_2) using MAX-MIN Law => degree MF output
				In case: 2 input and 1 output
				"If clause":   if AND => using MIN()
				"If clause":   if OR  => using MAX()
				"Then clause":           using MIN()
Step 5: Defuzzification: caculate output via Loop all rule list
				Using Average-Height method
				=> real output value
*/

#define NUMBER_RULE					49  //so luong rule
#define NUMBER_NAME					30  //so luong char cua string_name
#define MAXIMUM(a,b)								(float)( (a > b) ? a : b )
#define MINIMUM(a,b)								(float)( (a < b) ? a : b )
	
#define TYPE_INPUT_GOCLECH 					 0
#define TYPE_INPUT_GOCLECH_DOT 			 1
#define TYPE_OUTPUT_ValuePWMControl  2
	


#define GocLech_maxXD 							 (float)45.0
#define GocLech_minXD 							(float)-45.0

#define GocLech_dot_maxXD 					 (float)20.0
#define GocLech_dot_minXD 					(float)-20.0

#define ValuePWMControl_maxXD 			 (float)120.0
#define ValuePWMControl_minXD 			(float)-120.0
	
#define LIMIT_OUTPUT_FUZZY					(float)100.0
#define VALUE_SANG_LED_BY_MPU6050		(float)7.5


//-------MEMBERSHIP FUNCTION--------------------------------
//data struct cua MembershipFunction (FuzzySet)
typedef struct _MF{
	//char  name[30];       			  name of system input/output       */
	int 			type;									/*  input MF or output MF       */
	float 		h;										/*  degree cua MF       */
	float 		minXacDinh;
	float 		maxXacDinh;
	float 		a;										/*  A position of MF */
	float 		b;										/*  B position of MF */
	float 		c;										/*  C position of MF */
	float 		d;										/*  D position of MF */
} MF;

//--------Fuzzy Rule----------------------------------
// Data struct cua 1 Rule. Rule: dinh nghia cac luat trong fuzzy system
typedef struct _MF_rule{
	//char 			name[30]; 								//		 ten cua rule if A then B  */
	int 			rule_index_number;						//index cua luat. vi du luat thu 5
	MF  			* inGocLech;										/*membership function input cua Rule (o day la A)*/
	MF     		* inGocLech_dot;								/*membership function input cua Rule (o day la B)*/
	MF  			* outValuePWMControl;         	/*membership function outpt cua Rule (o day la C)*/
	float 		h;        									/* do cao(strength) cua Rule hoac gia tri cua membership output MF* out */
	float 		y;    											/*gia tri trung binh theo truc x cua output MF vd: (0+100)/2 */		
} MF_rule;


//-------FuzzyController--------------------------------
//gom co rollFuzzyControl va pitchFuzzyControl
typedef struct _FuzzyController{
	//char	  	name[30];       							 //roll / pitch /yaw    */
	MF   			inGocLech[7];									//input do lech goc error
	MF   			inGocLech_dot[7];							//input dao ham` do lech goc
	MF   			outValuePWMControl[7];					//output MF gia tri PWM +/-
	float 		pre_GocLech;  								//tinh dao ham = ( pre - current)/dt;
	float 		output;												//gia tri output control
	MF_rule   fuzzy_rules[NUMBER_RULE];	//rule fuzzy
} FuzzyController;


//--------------------------------------------------------------
void setABCD_MF(MF *mf, float a, float b, float c, float d, int type, float minXD, float maxXD)
{
	mf->a = a;
	mf->b = b;
	mf->c = c;
	mf->d = d;
	mf->type = type;
	mf->minXacDinh = minXD;
	mf->maxXacDinh = maxXD;
	mf->h = 0;
}

/*--------MO HOA - FUZZIFY-------------------------------
Mo` hoa' 1 Membership function. tu` gia tri ro~ x, ta tinh ra strength(degree or h) cua MF [0...1]
Tong quat cho hinh tam giac' va hinh` thang
|			B.................C
|		 /                   \
|		/                     \
|	 /                       \
..*.........................D.....-> x 
*/
void fuzzification(float x, MF *mf )
{
	float value = 0;
	if(x <= mf->a)
	{
			if(mf->a == mf->b) 
				value = 1; 
			else 
				value = 0;
	}
	else if(x >= mf->d )
	{
			if(mf->d == mf->c) 
				value = 1; 
			else 
				value = 0;
	}
	else if(x > mf->a && x < mf->b)
		value = ((float)(x - mf->a))/(mf->b - mf->a);
	else if(x >= mf->b && x <= mf->c )
		value = 1;
	else if( x > mf->c && x < mf->d)
		value= ((float)(mf->d - x))/(mf->d - mf->c);
	else 
		value = 0;
	
	if(value <= 0) 
		mf->h = 0;
	else if(value >= 1) 
		mf->h = 1;
	else 
		mf->h = (float)value;
}




//----------------ONE RULE------------ 
//Set input, output cho mot Rule
void setOneRule(MF_rule* rule, MF * inGocLech, MF * inGocLech_dot, MF * outValuePWMControl, int rule_index_number)
{
	rule->inGocLech 						= inGocLech;
	rule->inGocLech_dot					= inGocLech_dot;
	rule->outValuePWMControl		= outValuePWMControl;
	rule->rule_index_number     = rule_index_number;
	rule->h = 0;
	rule->y = 0;
}

//tinh output cua moi~ Rule (tinh h va tinh avg_x)
//	dung vong for loop het NUMBER_RULE,//	voi moi Rule, dung MAX or MIN => h,//	tinh avg_x = (left+right)/2
void calcule_H_and_Y_PerRule(float x, MF_rule   * rule )
{
		float temp=0;
		if(!rule) return;
		//rule->h = (float) MAXIMUM(  MINIMUM(rule->inGocLech->h, rule->inGocLech_dot->h ) , 0);
	//loai bo inGoclech_dot	
	rule->h = (float) MAXIMUM(  MINIMUM(rule->inGocLech->h, 1 ) , 0);
	
		if(rule->outValuePWMControl->a == rule->outValuePWMControl->b)
		{
			temp = ((float)(rule->outValuePWMControl->c + rule->outValuePWMControl->d)/2);
			rule->y = (float)( (float)(rule->outValuePWMControl->a + temp)/2) ;
		}
		else if(rule->outValuePWMControl->c == rule->outValuePWMControl->d)
		{
			temp = ((float)(rule->outValuePWMControl->a + rule->outValuePWMControl->b)/2);
			rule->y = (float)( (float)(rule->outValuePWMControl->d + temp)/2) ;
		}
		else
			rule->y = (float)( (float)(rule->outValuePWMControl->a + rule->outValuePWMControl->d)/2) ;
		
		if(rule->y == 0)
		{
			if(x < 0)
			{
				rule->y = (float)(rule->outValuePWMControl->a/2);
			}
			else if(x>0)
			{
				rule->y = (float)(rule->outValuePWMControl->d/2);
			}
				
		}
}







//-----------------------------------------------------------------------
