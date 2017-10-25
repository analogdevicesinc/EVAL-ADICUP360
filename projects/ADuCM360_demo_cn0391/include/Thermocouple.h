/*!
 *****************************************************************************
 * @file:    Thermocouple.h
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef THERMOCOUPLE_H_
#define THERMOCOUPLE_H_

#ifdef  __cplusplus
extern "C" {
#endif

enum
{
        TYPE_T = 0x00,
        TYPE_J,
        TYPE_K,
        TYPE_E,
        TYPE_S,
        TYPE_R,
        TYPE_N,
        TYPE_B
};

typedef struct _temp_range
{
     float neg_temp[16];        /* negative temperature */
     float pos_temp1[12];       /* positive temperature first limit*/
     float pos_temp2[12];       /* positive temperature second limit*/
     float neg_voltage[12];    /* negative voltage */
     float pos_voltage1[11];   /* positive voltage first limit*/
     float pos_voltage2[11];   /* positive voltage second limit*/
     float pos_voltage3[11];   /* positive voltage third limit*/
}temp_range;

const temp_range thPolyCoeff[] =
{
     // TYPE_T
{ {0.0, 0.387481063e-1f, 0.441944343e-4f, 0.118443231e-6f, 0.200329735e-7f, 0.901380195e-9f, 0.226511565e-10f, 0.360711542e-12f, 0.384939398e-14f, 0.282135219e-16f, 0.142515947e-18f, 0.487686622e-21f, 0.107955392e-23f, 0.139450270e-26f, 0.797951539e-30f, 1.0f},
    {0.0, 0.387481063e-1f, 0.332922278e-4f, 0.206182434e-6f, -0.218822568e-8f, 0.109968809e-10f, -0.308157587e-13f, 0.454791352e-16f, -0.275129016e-19f, 1.0f},
    {1.0f},
    {0.0, 2.5949192e+1f, -2.1316967e-1f, 7.9018692e-1f, 4.2527777e-1f, 1.3304473e-1f, 2.0241446e-2f, 1.2668171e-3f, 1.0f},
    {0.0, 2.592800e+1f, -7.602961e-1f, 4.637791e-2f, -2.165394e-3f, 6.048144e-5f, -7.293422e-7f, 1.0f},
    {1.0f},
    {1.0f}

  },
     // TYPE_J
  { {0.0, 0.503811878e-1f, 0.304758369e-4f, -0.856810657e-7f, 0.132281952e-9f, -0.170529583e-12f, 0.209480906e-15f, -0.125383953e-18f, 0.156317256e-22f, 1.0f},
    {0.296456256e+3f, -0.149761277e+1f, 0.317871039e-2f, -0.318476867e-5f, 0.157208190e-8f, -0.306913690e-12f, 1.0f},
    {1.0f},
    {0.0, 1.9528268e+1f, -1.2286185e+0f, -1.0752178e+0f, -5.9086933e-1f, -1.7256713e-1f, -2.8131513e-2f, -2.3963370e-3f, -8.3823321e-5f, 1.0f},
    {0.0, 1.978425e+1f, -2.001204e-1f, 1.036969e-2f, -2.549687e-4f, 3.585153e-6f, -5.344285e-8f, 5.099890e-10f, 0.0,  1.0f},
    {-3.11358187e+3f, 3.00543684e+2f, -9.94773230e+0f, 1.70276630e-1f, -1.43033468e-3f, 4.73886084e-6f, 1.0f},
    {1.0f}

  },
     // TYPE_K
  { {0.0, 0.394501280e-1f, 0.236223735e-4f, -0.328589067e-6f, -0.499048287e-8f, -0.675090591e-10f, -0.574103274e-12f, -0.310888728e-14f, -0.104516093e-16f, -0.198892668e-19f, -0.163226974e-22f,1.0f},
    {-0.176004136e-1f, 0.389212049e-1f, 0.185587700e-4f, -0.994575928e-7f, 0.318409457e-9f, -0.560728448e-12f, 0.560750590e-15f, -0.320207200e-18f, 0.971511471e-22f, -0.121047212e-25f, 1.0f},
    {1.0f},
    {0.0, 2.5173462e+1f, -1.1662878e+0f, -1.0833638e+0f, -8.9773540e-1f, -3.7342377e-1f, -8.6632643e-2f, -1.0450598e-2f, -5.19205771e-4f, 0.0, 1.0f},
    {0.0, 2.508355e+1f, 7.860106e-2f, -2.503131e-1f, 8.315270e-2f, -1.228034e-2f, 9.804036e-4f, -4.413030e-5f, 1.057734e-6f, -1.052755e-8f, 1.0f},
    {-1.318058e+2f, 4.830222e+1f, -1.646031e+0f,  5.464731e-2f, -9.650715e-4f, 8.802193e-6f, -3.110810e-8f, 1.0f},
    {1.0f}

  },
     // TYPE_E
  { {0.0, 0.586655087e-1f, 0.454109771e-4f, -0.779980486e-6f, -0.258001608e-7f, -0.594525830e-9f, -0.932140586e-11f, -0.102876055e-12f, -0.803701236e-15f, -0.439794973e-17f, -0.164147763e-19f, -0.396736195e-22f, -0.558273287e-25f, -0.346578420e-28f, 1.0f},
    {0.0, 0.586655087e-1f, 0.450322755e-4f, 0.289084072e-7f, -0.330568966e-9f, 0.650244032e-12f, -0.191974955e-15f, -0.125366004e-17f, 0.214892175e-20f, -0.143880417e-23f, 0.359608994e-27f, 1.0f},
    {1.0f},
    {0.0, 1.6977288e+1f, -4.3514970e-1f, -1.5859697e-1f, -9.2502871e-2f, -2.6084314e-2f, -4.1360199e-3f, -3.4034030e-4f, -1.1564890e-5f, 0.0, 1.0f},
    {0.0, 1.7057035e+1f, -2.3301759e-1f, 6.5435585e-3f, -7.3562749e-5f, -1.7896001e-6f, 8.4036165e-8f, -1.3735879e-9f, 1.0629823e-11f, -3.2447087e-14, 1.0f},
    {1.0f},
    {1.0f}

  },
   // TYPE_S
  { {0.0, 0.540313308e-2f, 0.125934289e-4f, -0.232477968e-7f, 0.3220288238e-10f, -0.331465196e-13f, 0.255744251e-16f, -0.125068871e-19f,  0.271443176e-23f, 1.0f},
    {0.132900444e+1f, 0.334509311e-2f, 0.654805192e-5f, -0.164856259e-8f, 0.129989605e-13f, 1.0f},
    {0.146628232e+3f, -0.258430516e+0f, 0.163693574e-3f, -0.330439046e-7f, -0.943223690e-14f, 1.0f},
    {0.0, 1.84949460e+2f, -8.00504062e+1f, 1.02237430e+2f, -1.52248592e+2f, 1.88821343e+2f, -1.59085941e+2f, 8.23027880e+1f,  -2.34181944e+1f, 2.79786260e+0f, 1.0f},
    {1.291507177e+1f, 1.466298863e+2f, -1.534713402e+1f, 3.145945973e+0f, -4.163257839e-1f, 3.187963771e-2f, -1.291637500e-3f, 2.183475087e-5f, -1.447379511e-7f, 8.211272125e-9f, 1.0f},
    {-8.087801117e+1f, 1.621573104e+2f, -8.536869453e+0f,  4.7196869763e-1f, -1.441693666e-2f, 2.081618890e-4f, 1.0f},
    {5.333875126e+4f, -1.235892298e+4f, 1.092657613e+3f,  -4.265693686e+1f, 6.247205420e-1f, 1.0f}
  },

     // TYPE_R
  { {0.0, 0.528961729e-2f, 0.139166589e-4f, -0.238855693e-7f, 0.356916001e-10f, -0.462347666e-13f, 0.500777441e-16f, -0.373105886e-19f,  0.157716482e-22f, -0.281038625e-26f,1.0f},
    {0.29515792e+1f, -0.252061251e-2f, 0.159564501e-4f, -0.764085947e-8f, 0.205305291e-11f, -0.293359668e-15f, 1.0f},
    {0.152232118e+3f, -0.268819888e+0f, 0.171280280e-3f, -0.345895706e-7f, -0.934633971e-14f, 1.0f},
    {0.0, 1.8891380e+2f, -9.3835290e+1f, 1.3068619e+2f, -2.2703580e+2f, 3.5145659e+2f, -3.8953900e+2f, 2.8239471e+2f,  -1.2607281e+2f, 3.1353611e+1f, -3.3187769e+0f, 1.0f},
    {1.334584505e+1f, 1.472644573e+2f, -1.844024844e+1f, 4.031129726e+0f, -6.249428360e-1f, 6.468412046e-2f, -4.458750426e-3f, 1.994710149e-4f, -5.313401790e-6f, 6.481976217e-8f, 1.0f},
    {-8.199599416e+1f, 1.553962042e+2f, -8.342197663e+0f,  4.279433549e-1f, -1.191577910e-2f, 1.492290091e-4f, 1.0f},
    {3.406177836e+4f, -7.023729171e+3f, 5.582903813e+2f,  -1.952394635e+1f, 2.560740231e-1f, 1.0f}
  },

     // TYPE_N
  { {0.0, 0.261591059e-1f, 0.109574842e-4f, -0.938411115e-7f, -0.464120397e-10f, -0.675090591e-10f, -0.263033577e-11f, -0.226534380e-13f, -0.760893007e-16f, -0.934196678e-19f, 1.0f},
    {0.0, 0.259293946e-1f, 0.157101418e-4f, 0.438256272e-7f, -0.252611697e-9f, 0.643118193e-12f, -0.100634715e-14f, 0.997453389e-18f, -0.608632456e-21f, 0.208492293e-24f, -0.306821961e-28f, 1.0f},
    {1.0f},
    {0.0, 3.8436847e+1f, 1.1010485e+0f, 5.2229312e+0f, 7.2060525e+0f, 5.8488586e+0f, 2.7754916e+0f, 7.7075166e-1f, 1.1582665e-1f, 7.3138868e-3f, 1.0f},
    {0.0, 3.86896e+1f, -1.08267e+0f, 4.70205e-2f, -2.12169e-6f, -1.17272e-4f, 5.39280e-6f, -7.98156e-8f, 1.0f},
    {1.972485e+1f, 3.300943e+1f, -3.915159e-1f,  9.855391e-3f, -1.274371e-4f, 7.767022e-7f, 1.0f},
    {1.0f}

  },
     // TYPE_B
  { {0.0, -0.246508183e-3f, 0.590404211e-5f, -0.13257931e-8f, 0.156682919e-11f, -0.169445292e-14f, 0.629903470e-18f, 1.0f},
    {-0.389381686e+1f, 0.285717474e-1f, -0.848851047e-4f, 0.157852801e-6f, -0.168353448e-9f, 0.111097940e-12f, -0.445154310e-16f, 0.989756408e-20f, -0.937913302e-24f, 1.0f},
    {1.0f},
    {9.8423321e+1f, 6.9971500e+2f, -8.4765304e+2f, 1.0052644e+3f, -8.3345952e+2f, 4.5508542e+2f, -1.5523037e+2f, 2.9886750e+1f, -2.4742860e+0f, 1.0f},
    {2.1315071e+2f, 2.8510504e+2f, -5.2742887e+1f, 9.9160804e+0f, -1.2965303e+0f, 1.1195870e-1f, -6.0625199e-3f, 1.8661696e-4f, -2.4878585e-6f, 1.0f},
    {1.0f},
    {1.0f}
  }

};

const float cjTempRange[][4] =
{
    { -200, 0, 400},                                    // TYPE_T
    { -210, 760, 1200},                                 // TYPE_J
    { -270, 0, 1372},                                   // TYPE_K
    { -270, 0, 1000},                                   // TYPE_E
    { -50, 1064.18, 1664.5, 1768.1},                    // TYPE_S
    { -50, 1064.18, 1664.5, 1768.1},                    // TYPE_R
    { -270, 0, 1300},                                   // TYPE_N
    { 0, 630.615, 1820}                                 // TYPE_B

};

const int thTempRange[][2] =
{
    { -200, 400},                                    // TYPE_T
    { -210, 1200},                                   // TYPE_J
    { -200, 1372},                                   // TYPE_K
    { -200, 1000},                                   // TYPE_E
    { -50, 1768},                                    // TYPE_S
    { -50, 1768},                                    // TYPE_R
    { -200, 1300},                                   // TYPE_N
    { 250, 1820}                                     // TYPE_B

};


const float thVoltageRange[][5] =
{
    { -5.603, 0, 20.872},                               // TYPE_T
    { -8.095, 0, 42.919, 69.553},                       // TYPE_J
    { -5.891, 0, 20.644, 54.886},                       // TYPE_K
    { -8.825, 0, 76.373},                               // TYPE_E
    { -0.235, 1.874, 10.332, 17.536, 18.693},           // TYPE_S
    { -0.226, 1.923, 11.361, 19.739, 21.103},           // TYPE_R
    { -3.990, 0, 20.613, 47.513},                       // TYPE_N
    { 0.291, 2.431, 13.820}                             // TYPE_B

};

/* Coefficients for thermocouple type K -> temp > 0°C */
#define COEFF_K_A0               0.1185976
#define COEFF_K_A1               -0.1183432e-3f
#define COEFF_K_A2               0.1269686e+3f


/***************************Configuration*************************/
//Select default thermocouple types used for each channel
#define   P1_TYPE           TYPE_J
#define   P2_TYPE           TYPE_K
#define   P3_TYPE           TYPE_T
#define   P4_TYPE           TYPE_E

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif /* THERMOCOUPLE_H_ */
