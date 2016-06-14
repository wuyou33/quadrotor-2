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
	


#define GocLech_maxXD 							 (float)30
#define GocLech_minXD 							(float)-30

#define GocLech_dot_maxXD 					 (float)10
#define GocLech_dot_minXD 					(float)-10

#define ValuePWMControl_maxXD 			 (float)200
#define ValuePWMControl_minXD 			(float)-200


//-------MEMBERSHIP FUNCTION--------------------------------
//data struct cua MembershipFunction (FuzzySet or Tap Mo`)
typedef struct _MF{
	//char  name[30];       			  name of system input/output       */
	int 			type;									/*  loai gi input hay output       */
	float 		h;										/*  degree cua MF       */
	float 		minXacDinh;
	float 		maxXacDinh;
	float 		a;										/*  gia tri diem A trong hinh thang/tam giac */
	float 		b;										/*  gia tri diem B trong hinh thang/tam giac */
	float 		c;										/*  gia tri diem C trong hinh thang/tam giac */
	float 		d;										/*  gia tri diem D trong hinh thang/tam giac */
} MF;

//--------Fuzzy Rule----------------------------------
// Data struct cua 1 Rule. Rule: dinh nghia cac luat trong fuzzy system
typedef struct _MF_rule{
	//char 			name[30]; 								//		 ten cua rule if A then B  */
	int 			rule_index_number;						//index cua luat. vi du luat thu 5
	MF  			inGocLech;										/*membership function input cua Rule (o day la A)*/
	MF     		inGocLech_dot;								/*membership function input cua Rule (o day la B)*/
	MF  			outValuePWMControl;         	/*membership function outpt cua Rule (o day la C)*/
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
Mo` hoa' 1 Membership function. tu` gia tri ro~ x, ta tinh ra strength(degree or h) cua Membership Function [0...1]
Tong quat cho hinh tam giac' va hinh` thang
|			B.................C
|		 /                   \
|		/                     \
|	 /                       \
..*.........................D................-> x 
*/
void fuzzification(MF *mf, float x)
{
	float value = 0;
	if(x <= mf->a)
		value= 0;
	
	else if(x >= mf->d )
		value = 0;
	
	else if(x > mf->a && x < mf->b)
		value = (float)(x - mf->a)/(mf->b - mf->a);
	
	else if(x >= mf->b && x <= mf->c )
		value = 1;
	
	else if( x > mf->c && x < mf->d)
		value= (float)(mf->d - x)/(mf->d - mf->c);
	
	else 
		value = 0;	
	
	if(value <= 0) 
		mf->h = 0;
	else if(value >= 1) 
		mf->h = 1;
	else 
		mf->h = value;
}




//----------------ONE RULE------------ 
//Set input, output cho mot Rule
void setOneRule(MF_rule* rule, MF inGocLech, MF inGocLech_dot, MF outValuePWMControl, int rule_index_number)
{
	rule->inGocLech 						= inGocLech;
	rule->inGocLech_dot					= inGocLech_dot;
	rule->outValuePWMControl		= outValuePWMControl;
	rule->rule_index_number     = rule_index_number;
	rule->h = 0;
}

//tinh output cua moi~ Rule (tinh h va tinh avg_x)
//	dung vong for loop het NUMBER_RULE
//	voi moi Rule, dung MAX or MIN => h
//	tinh avg_x = (left+right)/2
void calcule_H_and_Y_PerRule(MF_rule   * rule )
{
		if(!rule) return;
		rule->h = (float) MAXIMUM(  MINIMUM(rule->inGocLech.h, rule->inGocLech_dot.h ) , 0);
		rule->y = (float)( (rule->outValuePWMControl.a + rule->outValuePWMControl.d)/2) ;
}







//-----------------------------------------------------------------------
