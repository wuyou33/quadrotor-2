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
//---------------------------------------
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
typedef struct{
	float a;
	float b;
	float c;
	float d;
			
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



/*--------Mo` Hoa' ngo~ vao`-------------------------------
Fuzzify all input values into fuzzy membership functions.
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
float fuzzify(float x, float left, float left_top, float right_top, float right)
{
	float value = 0;
	if(x <= left)
		value= 0;
	else if(x > left && x < left_top)
		value= (float)(x-left)/(left_top - left);
	else if(x >= left_top && x <= right_top)
		value= 1;
	else if( x > right_top && x < right)
		value= (float)(right-x)/(right - right_top);
	else if(x >= right)
		value= 0;
	else 
		value= 0;	
	
	if(value <= 0) return 0;
	else if(value>=1) return 1;
	else return value;
}


//--------Giai Mo`-------------------------------
//De-fuzzify the fuzzy output functions to get "crisp" output values.
float defuzzify(float x, float a, float b, float c, float d)
{
	float dom;
	return dom;
}