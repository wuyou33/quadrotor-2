#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
/*
Fuzzy logic
Author: nguyen
Date : 13/5/2016
Description: 
	MF_in
	MF_out
	MF_rule
	Fuzzify : mo` hoa' ngo~ vao (bien ngon ngu)
	Defuzzify : giai`~ mo`

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


//-------MEMBERSHIP FUNCTION--------------------------------
typedef struct {
	char  name[30];       		/*  name of system input/output       */
	float type;								/*  loai gi input hay output       */
	float h;									/*  degree cua MF       */
	float mienXDmax;
	float mienXDmin;
	float left;								/*  gia tri diem A trong hinh thang/tam giac */
	float left_top;						/*  gia tri diem B trong hinh thang/tam giac */
	float right_top;					/*  gia tri diem C trong hinh thang/tam giac */
	float right;							/*  gia tri diem D trong hinh thang/tam giac */
} MF;

//-----RULE FUZZY RULE----------------------------------
typedef struct{
	char name[30]; 						/* ten cua rule if A then B  */
	MF* in;										/*membership function input cua Rule (o day la A)*/
	MF* out;         					/*membership function outpt cua Rule (o day la B)*/
	float h;        					/* do cao(strength) cua Rule hoac gia tri cua membership output MF* out */
	float avg_x;    					/*gia tri trung binh theo truc x cua output MF vd: (0+100)/2 */		
} MF_rule;


//-------FuzzyController--------------------------------
typedef struct {
	char  name[30];       		/*  roll / pitch /yaw    */
	MF * in[7];
	MF * out[5];
	MF_rule * rules[NUMBER_RULE];
} FuzzyController;


//--------------------------------------------------------------
void setLeftRight_ForMF(MF *mf, float left, float left_top, float right_top, float right)
{
	mf->left = left;
	mf->left_top = left_top;
	mf->right = right;
	mf->right_top = right_top;
}
void setName_ForMF(MF *mf, char* name )
{
	strcpy(mf->name, name);
	//strlcpy(mf->name, name, sizeof(name));
}
void setType_ForMF(MF *mf, float type)
{
	mf->type = type;
}
void setMienXD_ForMF(MF *mf, float mienXDmax, float mienXDmin)
{
	mf->mienXDmax = mienXDmax;
	mf->mienXDmin = mienXDmin;
}



/*--------MO HOA - FUZZIFY-------------------------------
Mo` hoa' 1 Membership function. tu` gia tri ro~ x, ta tinh ra strength cua Membership Function [0...1]
Tong quat cho hinh tam giac' va hinh` thang
 left_top(B)         right_top(C)
|
|			*.................*
|		 /                   \
|		/                     \
|	 /                       \
..*.........................*................-> x 
left(A)                     right(D)
x: gia tri ro~
*/
void fuzzify(float x, MF *mf)
{
	float value = 0;
	if(x <= mf->left)
		value= 0;
	else if(x > mf->left && x < mf->left_top)
		value = (float)(x - mf->left)/(mf->left_top - mf->left);
	else if(x >= mf->left_top && x <= mf->right_top)
		value = 1;
	else if( x > mf->right_top && x < mf->right)
		value= (float)(mf->right-x)/(mf->right - mf->right_top);
	else if(x >= mf->right)
		value = 0;
	else 
		value = 0;	
	
	if(value <= 0) mf->h = 0;
	else if(value>=1) mf->h = 1;
	else mf->h = value;
}

// => khi co gia tri ro~ thi` phai fuzzify tat ca cac MF: MF small, MF zero, MF big,....
// fuzzify(roll, RatNho); //roll la goc lech roll
// fuzzify(roll, Nho); //roll la goc lech roll
// fuzzify(roll, NhoVua); //roll la goc lech roll
// fuzzify(roll, Zero); //roll la goc lech roll




//----------------APPLY RULE -------------------------------
void setOneRule(MF_rule* rule, MF* in, MF* out)
{
	rule->in = in;
	rule->out= out;
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
void calcule_H_and_Avg_x_ForEachRule(MF_rule   (*rules)[NUMBER_RULE] )
{
	int i;
	for (i=0; i <= NUMBER_RULE; i++) 
	{ 
		if(!rules[i]) continue;
		rules[i]->h = 		(float)MINIMUM(rules[i]->in->h, rules[i]->out->h);
		rules[i]->avg_x = (float)( (rules[i]->out->left + rules[i]->out->right)/2) ;
	}
}




//--------GIAI MO` De-fuzzify-------------------------------
//De-fuzzify the fuzzy output functions to get "crisp" output values.
float defuzzify(MF_rule   (*rules)[NUMBER_RULE] )
{
	float output = 0;
	int i;
	float sum_h = 0; 					//tong tat ca cac strength( degree) cua cac MF thanh phan
	float total_avg_x = 0; 		//tong tat ca cac x(hoanh do) trong tam cua MF thanh phan
	for (i=0; i <= NUMBER_RULE; i++) 
	{ 
		if(!rules[i]) continue;
		sum_h = 			(float)sum_h + rules[i]->h;
		total_avg_x = (float)total_avg_x + (float)rules[i]->avg_x * rules[i]->h;
	}
	output = (float)(total_avg_x/sum_h);
	return output; //PWM + hoac - mot gia tri output
}


//-----------------------------------------------------------------------
