/*
 * fuzzy.h
 *
 *  Created on: Mar 30, 2024
 *      Author: lehuu
 */

#ifndef INC_FUZZY_H_
#define INC_FUZZY_H_

float max(int num_args, ...);
float min(float a, float b);
//float max(float a, float b);
float mfTriang(float x, float a, float b, float c);
float mfTrap(float x, float a, float b, float c, float d);
float run_fuzzy(float x1, float x2);
void limit_range(float *x);


#endif /* INC_FUZZY_H_ */
