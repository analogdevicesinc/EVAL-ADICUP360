/***************************************************************************//**
 *   @file   RTD.cpp
 *   @brief  Implementation of RTD.cpp
 *   @author
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <ADuCM360.h>

#include <cmath>
#include <cfloat>

#include <hal/RTD.h>

#include <cassert>

#define ORDER 4

#define FLOAT_TYPE float

#define MAX_T 850.f
#define MIN_T -200.f

#define MIN_R 0.1852008f
#define MAX_R 3.90481125f

FLOAT_TYPE rtd_coeff[] = {1.e0l, 3.9083e-3l, -5.775e-7l, 4.183e-10l, -4.183e-12l};

/* num_coeff[] is numerator polynomial coefficient list
 * den_ceoff[] is denominator polynomial coefficient list
 * ORDER defined rational polynomial precision
 */

#if ORDER == 4

//effective bits 15.616
FLOAT_TYPE num_coeff[] = {-2.42580348918581e2l, 4.43575581447635e2l, -8.6570608699616e2l, 7.34933695653457e2l, -7.02396810482965e1l};
FLOAT_TYPE den_coeff[] = {1.e0l, -8.87493143817912e-1l, 2.75291524634514e0l, -3.88521936154463e-1l, 9.08722079164108e-3l};

#endif

#if ORDER == 5

//effective bits 17.8245
FLOAT_TYPE num_coeff[] = {-2.41711451348096e2l, 1.54212010305297e2l, 5.54970679760118e2l, -1.26796609113434e3l, 8.56166744043382e2l, -5.56764891254276e1l};
FLOAT_TYPE den_coeff[] = {1.e0l, 2.63302632747995e-1l, -1.84510086458895e0l, 3.2746227792845e0l, -3.25653791878951e-1l, 3.44022334655219e-3l};

#endif

#if ORDER == 6

//effective bits 19.82
FLOAT_TYPE num_coeff[] = {-2.42142843978731e2l, 5.92083047602897e2l, -9.25895464531705e2l, 1.39340166396238e3l,
			  -1.47225489959494e3l, 7.09664483583688e2l, -5.48565127920454e1l
			  };
FLOAT_TYPE den_coeff[] = {1.e0l, -1.51978059117102e0l, 2.48277654313265e0l, -3.47112904866882e0l, 2.80056808288615e0l,
			  -3.19022897847929e-1l, 6.25422836057902e-3l
			  };

#endif

#if ORDER == 7

//effective bits 21.5
FLOAT_TYPE num_coeff[] = {-2.41929621588221e2l, 4.82396434080762e2l, -1.67121893244441e1l, -1.37315140916645e3l,
			  2.81292843561081e3l, -2.67151104549731e3l, 1.08281818377997e3l, -7.48389592163684e1l
			 };
FLOAT_TYPE den_coeff[] = {1.e0l, -1.08173661281902e0l, -7.63181691057539e-1l, 4.60429829752353e0l, -6.86578653181876e0l,
			  4.30546544990275e0l, -4.37253401604816e-1l, 7.00961785543309e-3l
			 };

#endif

#if ORDER == 8

//effective bits 23.337
FLOAT_TYPE num_coeff[] = {-2.42050820471513e2l, 9.03922568256792e2l, -1.59215856239e3l, 2.05579651580702e3l,
			  -2.5023311275312e3l, 2.58715968247352e3l, -1.72836289632771e3l, 5.56615054519548e2l,
			  -3.85904217448201e1l
			  };
FLOAT_TYPE den_coeff[] = {1.e0l, -2.81393474700247e0l, 4.07953003378532e0l, -4.96524658655131e0l, 5.95307465876528e0l,
			  -5.20533093922899e0l, 2.29944525926975e0l, -2.31214523979689e-1l, 4.3399532290543e-3l
			  };

#endif

#if ORDER == 9

//effective bits 24.6
FLOAT_TYPE num_coeff[] = {-2.41968339549439e2l, 2.91135402145953e2l, 1.20532207372212e3l, -4.6631158165784e3l,
			  9.54173545500971e3l, -1.408246803981e4l, 1.4213332684621e4l, -8.59797958055946e3l,
			  2.49954894344233e3l, -1.65542792821347e2l
			 };
FLOAT_TYPE den_coeff[] = {1.e0l, -2.89485047831221e-1l, -5.08894034154434e0l, 1.42530537624761e1l, -2.58291119118538e1l,
			  3.35473327796638e1l, -2.67543757779824e1l, 1.04025522724728e1l, -9.94173619261334e-1l,
			  1.76585466116337e-2l
			 };

#endif

/*
 * temperature to resistance convert based on IEC751/ITS-90
 * temperature t in celsius degree
 * resistance normalized to 1 at 0°C
 *
 */

float temp2res(float t)
{
	assert(t >= MIN_T && t <= MAX_T);

	FLOAT_TYPE res = rtd_coeff[t >= 0 ? 2 : 4];

	for (int i = (t >= 0 ? 1 : 3); i >= 0; --i) {
		res = res * t + rtd_coeff[i];
	}

	return (res);
}

/*
 * resistance to temperature convert based on rational polynomial
 * resistance r normalized to 1 at 0°C
 * temperature in celsius degree
 *
 */

float res2temp(float r)
{
	assert(r >= MIN_R && r <= MAX_R);

	FLOAT_TYPE num_poly = num_coeff[sizeof(num_coeff) / sizeof(num_coeff[0]) - 1];
	FLOAT_TYPE den_poly = den_coeff[sizeof(den_coeff) / sizeof(den_coeff[0]) - 1];

	for (int i = (sizeof(num_coeff) / sizeof(num_coeff[0]) - 2); i >= 0; --i) {
		num_poly = num_poly * r + num_coeff[i];
	}

	for (int i = (sizeof(den_coeff) / sizeof(den_coeff[0]) - 2); i >= 0; --i) {
		den_poly = den_poly * r + den_coeff[i];
	}

	return (num_poly / den_poly);
}
