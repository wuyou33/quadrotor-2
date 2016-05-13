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
#define MAXNAME 10 
//---------------------------------------
struct MF_in{
	char name[MAXNAME];        /*  name of system input/output       */
	float a;
	float b;
	float c;
	float d;
			
} MF_in;

//---------------------------------------
struct MF_out{
	float a;
	float b;
	float c;
	float d;
			
} MF_out;

//---------------------------------------
struct MF_rule{
	float a;
	float b;
	float c;
	float d;
			
} MF_rule;


//--------Mo` Hoa' ngo~ vao`-------------------------------
//Fuzzify all input values into fuzzy membership functions.
float fuzzify(float x, float a, float b, float c, float d)
{
	float dom;
	return dom;
}



//--------Giai Mo`-------------------------------
//De-fuzzify the fuzzy output functions to get "crisp" output values.
float defuzzify(float x, float a, float b, float c, float d)
{
	float dom;
	return dom;
}