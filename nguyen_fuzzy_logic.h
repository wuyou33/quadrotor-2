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

-------------------------------------------------
Viet tat:
	MF_in: membership function input
	MF_out: membership function output

-------------------------------------------------
Fuzzy Process(thuc hien):
	step1:
	step2:
	step3:
	step4:
*/

#define NUMBER_RULE					10
#define NUMBER_NAME					30  //so luong char cua string_name



//---------------------------------------
//MEMBERSHIP FUNCTION
typedef struct {
	char  name[30];       /*  name of system input/output       */
	float type;								/*  loai gi input hay output       */
	float value;							/*  degree cua MF       */
	float mienXDmax;
	float mienXDmin;
	float left;								/*  gia tri diem A trong hinh thang/tam giac */
	float left_top;						/*  gia tri diem B trong hinh thang/tam giac */
	float right_top;					/*  gia tri diem C trong hinh thang/tam giac */
	float right;							/*  gia tri diem D trong hinh thang/tam giac */
} MF;





//---------------------------------------
//RULE
typedef struct{
	char name[30]; /* ten cua rule if A then B  */
	MF* in;					/*membership function input cua Rule (o day la A)*/
	MF* out;         /*membership function outpt cua Rule (o day la B)*/
	float h;        /* do cao(strength) cua Rule hoac gia tri cua membership output MF* out */
	float avg_x;    /*gia tri trung binh theo truc x cua output MF vd: (0+100)/2 */		
} MF_rule;



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



/*--------MO HOA - Fuzzify-------------------------------
Mo` hoa' 1 Membership function. tu` gia tri ro~ x, ta tinh ra strength cua Membership Function [0...1]
Tong quat cho hinh tam giac' va hinh` thang

 left_top(B)         right_top(C)
			...................
		 /                   \
		/                     \
	 /                       \
	...........................
left(A)                     right(D)
x: gia tri ro~
*/
float fuzzify(float x, MF *mf)
{
	float value = 0;
	if(x <= mf->left)
		value= 0;
	else if(x > mf->left && x < mf->left_top)
		value= (float)(x - mf->left)/(mf->left_top - mf->left);
	else if(x >= mf->left_top && x <= mf->right_top)
		value= 1;
	else if( x > mf->right_top && x < mf->right)
		value= (float)(mf->right-x)/(mf->right - mf->right_top);
	else if(x >= mf->right)
		value= 0;
	else 
		value= 0;	
	
	if(value <= 0) return 0;
	else if(value>=1) return 1;
	else return value;
}

// => khi co gia tri ro~ thi` phai fuzzify tat ca cac MF: MF small, MF zero, MF big,....




//--------APPLY RULE vao`-------------------------------
//tinh output cua moi~ Rule (tinh h va tinh avg_x)
//	dung vong for loop het NUMBER_RULE
//	voi moi Rule, dung MAX or MIN => h
//	tinh avg_x = (left+right)/2
float applyRule(MF_rule   (*rules)[NUMBER_RULE] )
{
	float dom;
	return dom;
}




//--------GIAI MO` De-fuzzify-------------------------------
//De-fuzzify the fuzzy output functions to get "crisp" output values.
float defuzzify(MF_rule   (*rules)[NUMBER_RULE] )
{
	float output = 0;
	
	return output;
}