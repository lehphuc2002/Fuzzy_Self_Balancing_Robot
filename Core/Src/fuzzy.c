/*
 * fuzzy.c
 *
 *  Created on: Mar 30, 2024
 *      Author: lehuu
 */

#include "fuzzy.h"
#include <stdarg.h>

typedef struct {
	float NB, NS, ZE, PS, PB;
} theta_struct;

typedef struct {
	float NB, NS, ZE, PS, PB;
} thetadot_struct;

typedef struct {
	float NB, NM, NS, ZE, PS, PM, PB;
} uk_struct;

float min(float a, float b) {
	return (a < b) ? a : b;
}

float c1, c2, c3; // parameters theta
float d1, d2, d3; // parameters theta_dot

float max(int num_args, ...) // VARIABLE MUST BE FLOAT (EX: 70.0, 980.0 NOT 70 OR 980) , IF NOT FLOAT --> MAX FUNC WILL BE FALSE!!
{
   float max;
   float current;
   va_list ap;
   va_start(ap, num_args);
   for(int i = 0; i < num_args; i++)
   {
	   current = (float)va_arg(ap, double);  // Auto increment the pointer to receive the next argument (float) every call function
	   current += 0.0000001f; // Convert to float again :>
	   if(i == 0 || current > max)
	   {
		   max = current;
	   }
   }
   va_end(ap);

   return max;
}


float mfTriang(float x, float a, float b, float c) {
  float out_tri;

  if (x < a) {
	  out_tri = 0;
  } else if ((x >= a) && (x <= b)) {
	  out_tri = (x - a) / (b - a);
  } else if ((x >= b) && (x <= c)) {
	  out_tri = (c - x) / (c - b);
  } else {
	  out_tri = 0;
  }

  return out_tri;
}

float mfTrap(float x, float a, float b, float c, float d) {
  float out_trap;

  if (x < a) {
	  out_trap = 0;
  } else if ((x >= a) && (x <= b)) {
    if (a == b) {
    	out_trap = 1;
    } else {
    	out_trap = (x - a) / (b - a);
    }
  } else if ((x >= b) && (x <= c)) {
	  out_trap = 1;
  } else if ((x >= c) && (x <= d)) {
    if (c == d) {
    	out_trap = 1;
    } else {
    	out_trap = (d - x) / (d - c);
    }
  } else {
	  out_trap = 0;
  }

  return out_trap;
}

float run_fuzzy(float x1, float x2) {
  float out;
  float r[25]; // 25 value beta <=> 25 rule
  theta_struct theta;
  thetadot_struct theta_dot;
  uk_struct u_dot;

  // calculate alpha 1 (0->1)
  c1 = 0.3f;
  c2 = 0.22f;
  c3 = 0.18f;
  theta.NB = mfTrap(x1, -2, -1, -c1, -c2); // ve hinh dinh nghia cac gia tri ngon ngu cua bien NB,NS,... (theta)
  theta.NS = mfTrap(x1, -c1, -c2, -c3, -0.001);
  theta.ZE = mfTrap(x1, -c3, -0.001, c3, c2);
  theta.PS = mfTrap(x1, c3, c2, c1, 1);
  theta.PB = mfTrap(x1, c2, c1, 1, 2);

  // calculate alpha 2 (0->1)
  d1 = 0.25f;
  d2 = 0.05f;
  d3 = 0.6f;
  theta_dot.NB = mfTriang(x2, -2, -1, -d1); // // ve hinh dinh nghia cac gia tri ngon ngu cua bien NB,NS,... (thetadot)
  theta_dot.NS = mfTriang(x2, -d3, -d1, -d2);
  theta_dot.ZE = mfTrap(x2, -d1, -d2, d2, d1);
  theta_dot.PS = mfTriang(x2, d1, d2, d3);
  theta_dot.PB = mfTriang(x2, d2, d3, 1);

  // calculate beta (0->1) base on MAX-MIN, "and" => MIN

  r[0] = min(theta.NB, theta_dot.NB); // NB

  r[1] = min(theta.NS, theta_dot.NB); // NB

  r[2] = min(theta.ZE, theta_dot.NB); // NM

  r[3] = min(theta.PS, theta_dot.NB); // NS

  r[4] = min(theta.PB, theta_dot.NB); // ZE
  //-----------------------

  r[5] = min(theta.NB, theta_dot.NS); // NB

  r[6] = min(theta.NS, theta_dot.NS); // NM

  r[7] = min(theta.ZE, theta_dot.NS); // NS

  r[8] = min(theta.PS, theta_dot.NS); // ZE

  r[9] = min(theta.PB, theta_dot.NS); // PS
  //-----------------------

  r[10] = min(theta.NB, theta_dot.ZE); // NM

  r[11] = min(theta.NS, theta_dot.ZE); // NS

  r[12] = min(theta.ZE, theta_dot.ZE); // ZE

  r[13] = min(theta.PS, theta_dot.ZE); // PS

  r[14] = min(theta.PB, theta_dot.ZE); // PM
  //-----------------------
  r[15] = min(theta.NB, theta_dot.PS); // NS

	r[16] = min(theta.NS, theta_dot.PS); // ZE

	r[17] = min(theta.ZE, theta_dot.PS); // PS

	r[18] = min(theta.PS, theta_dot.PS); // PM

	r[19] = min(theta.PB, theta_dot.PS); // PB
	//-----------------------

	r[20] = min(theta.NB, theta_dot.PB); // ZE

	r[21] = min(theta.NS, theta_dot.PB); // PS

	r[22] = min(theta.ZE, theta_dot.PB); // PM

	r[23] = min(theta.PS, theta_dot.PB); // PB

	r[24] = min(theta.PB, theta_dot.PB); // PB

  // Sugeno fuzzy system
  /* NB: r0 r1 r5
   * NM: r2 r6 r10
   * NS: r3 r7 r11 r15
   * ZE: r4 r8 r12 r16 r20
   * PS: r9 r13 r17 r21
   * PM: r14 r18 r22
   * PB: r19 r23 r24
   */
  u_dot.NB = max(3, r[0], r[1], r[5]);
  u_dot.NM = max(3, r[2], r[6], r[10]);
  u_dot.NS = max(4, r[3], r[7], r[11], r[15]);
  u_dot.ZE = max(5, r[4], r[8], r[12], r[16], r[20]);
  u_dot.PS = max(4, r[9], r[13], r[17], r[21]);
  u_dot.PM = max(3, r[14], r[18], r[22]);
  u_dot.PB = max(3, r[19], r[23], r[24]);

  // weighted average defuzzification method
  float sum_beta;
  float sum_beta_y;
  sum_beta = u_dot.NB + u_dot.NM + u_dot.NS + u_dot.ZE + u_dot.PS + u_dot.PM + u_dot.PB;
  sum_beta_y = -1 * u_dot.NB + -0.75 * u_dot.NM + -0.45 * u_dot.NS + 0 * u_dot.ZE + 0.45 * u_dot.PS + 0.75 * u_dot.PM + 1 * u_dot.PB;
  out = sum_beta_y / sum_beta; // Homework3 :>

  return out;
}

void limit_range(float *x) {
  if (*x > 1)
    *x = 1;
  else if (*x < -1)
    *x = -1;
}
