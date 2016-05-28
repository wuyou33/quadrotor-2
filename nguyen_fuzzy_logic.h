#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
/*
Fuzzy logic
Author: nguyen
Date : 13/5/2016

Fuzzy process:
B1: xac dinh cac MF
B2: xac dinh cac rule
B3: tu gia tri ro~ x => degree cua tat ca MF in va MF out
B4: tu (degree of MF in + Rule[i] ) => degree of MF out 
		Neu menh de IF dung and => MIN()           //ap dung khi co 2 input
		Neu menh de IF dung or  => MAX()					 //ap dung khi co 2 input
		=> degree of menh de IF
		=> degree of menh de THEN (ap dung MIN() )
B5: lap qua tat ca Rule. tinh ra output (phuong phap trong tam trung binh)
		=>output		
*/

#define NUMBER_RULE					10  //so luong rule
#define NUMBER_NAME					30  //so luong char cua string_name
#define MAXIMUM(a,b)								(float)( (a > b) ? a : b )
#define MINIMUM(a,b)								(float)( (a < b) ? a : b )
	
#define TYPE_INPUT_GOCLECH 					0
#define TYPE_INPUT_GOCLECH_DOT 			1
#define TYPE_OUTPUT_ValuePWMControl 	2


//-------MEMBERSHIP FUNCTION--------------------------------
//data struct cua MembershipFunction (FuzzySet or Tap Mo`)
typedef struct {
	char  name[30];       			/*  name of system input/output       */
	int 	type;									/*  loai gi input hay output       */
	float h;										/*  degree cua MF       */
	float minXacDinh;
	float maxXacDinh;
	float a;										/*  gia tri diem A trong hinh thang/tam giac */
	float b;										/*  gia tri diem B trong hinh thang/tam giac */
	float c;										/*  gia tri diem C trong hinh thang/tam giac */
	float d;										/*  gia tri diem D trong hinh thang/tam giac */
} MF;

//--------Fuzzy Rule----------------------------------
// Data struct cua 1 Rule. Rule: dinh nghia cac luat trong fuzzy system
typedef struct{
	char name[30]; 										/* ten cua rule if A then B  */
	int rule_index_number;						//index cua luat. vi du luat thu 5
	MF* inGocLech;										/*membership function input cua Rule (o day la A)*/
	MF* inGocLech_dot;								/*membership function input cua Rule (o day la B)*/
	MF* outValuePWMControl;         	/*membership function outpt cua Rule (o day la C)*/
	float h;        									/* do cao(strength) cua Rule hoac gia tri cua membership output MF* out */
	float y;    											/*gia tri trung binh theo truc x cua output MF vd: (0+100)/2 */		
} MF_rule;


//-------FuzzyController--------------------------------
//gom co rollFuzzyControl va pitchFuzzyControl
typedef struct {
	char  name[30];       							/*  roll / pitch /yaw    */
	MF * inGocLech[7];									//input do lech goc error
	MF * inGocLech_dot[7];							//input dao ham` do lech goc
	MF * outValuePWMControl[7];					//output MF gia tri PWM +/-
	float realGocLech;  								//gia tri ro~ cua goc lech
	float realGocLech_dot;							//dao ham cua goc lech
	float output;												//gia tri output control
	MF_rule * fuzzy_rules[NUMBER_RULE];	//rule fuzzy
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
	else if(x > mf->a && x < mf->b)
		value = (float)(x - mf->a)/(mf->b - mf->a);
	else if(x >= mf->b && x <= mf->c )
		value = 1;
	else if( x > mf->c && x < mf->d)
		value= (float)(mf->d - x)/(mf->d - mf->c);
	else if(x >= mf->d )
		value = 0;
	else 
		value = 0;	
	
	if(value <= 0) mf->h = 0;
	else if(value >= 1) mf->h = 1;
	else mf->h = value;
}

// => khi co gia tri ro~ thi` phai fuzzify tat ca cac MF: MF small, MF zero, MF big,....
// fuzzify(roll, RatNho); //roll la goc lech roll
// fuzzify(roll, Nho); //roll la goc lech roll
// fuzzify(roll, NhoVua); //roll la goc lech roll
// fuzzify(roll, Zero); //roll la goc lech roll




//----------------ONE RULE------------ 
//Set input, output cho mot Rule
void setOneRule(MF_rule* rule, MF* inGocLech, MF* inGocLech_dot, MF* outValuePWMControl, int rule_index_number)
{
	rule->inGocLech 						= inGocLech;
	rule->inGocLech_dot					= inGocLech_dot;
	rule->outValuePWMControl		= outValuePWMControl;
	rule->rule_index_number = rule_index_number;
	rule->h = 0;
}

void applyRuleList(MF_rule   (*rules)[NUMBER_RULE])
{
	//setOneRule(rules[0], RatNho, RatNhanh); //neu goc lenh nho~ thi output PWM nhanh
	//setOneRule(rules[0], RatNho, RatNhanh); //neu goc lenh nho~ thi output PWM nhanh
	//setOneRule(rules[0], RatNho, RatNhanh); //neu goc lenh nho~ thi output PWM nhanh
}

//tinh output cua moi~ Rule (tinh h va tinh avg_x)
//	dung vong for loop het NUMBER_RULE
//	voi moi Rule, dung MAX or MIN => h
//	tinh avg_x = (left+right)/2
void calcule_H_and_Y_PerRule(MF_rule   (*rules)[NUMBER_RULE] )
{
	int i;
	for (i=0; i <= NUMBER_RULE; i++) 
	{ 
		if(!rules[i]) continue;
		rules[i]->h = 		(float)MAXIMUM(  MINIMUM(rules[i]->inGocLech->h, rules[i]->inGocLech_dot->h ) , 0);
		rules[i]->y = (float)( (rules[i]->outValuePWMControl->a + rules[i]->outValuePWMControl->d)/2) ;
	}
}




//--------GIAI MO` De-fuzzify-------------------------------
//De-fuzzify the fuzzy output functions to get "crisp" output values.
float defuzzification(MF_rule   (*rules)[NUMBER_RULE] )
{
	float output = 0;
	int i;
	float sum_h = 0; 					//tong tat ca cac strength( degree) cua cac MF thanh phan
	float total_y = 0; 				//tong tat ca cac x(hoanh do) trong tam cua MF thanh phan
	for (i=0; i <= NUMBER_RULE; i++) 
	{ 
		if(!rules[i]) continue;
		sum_h = 			(float)sum_h + rules[i]->h;
		total_y = (float)total_y + (float)rules[i]->y * rules[i]->h;
	}
	output = (float)(total_y/sum_h);
	return output; //PWM + hoac - mot gia tri output
}


//-----------------------------------------------------------------------
