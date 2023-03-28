/*************************************************************************
 *     SAP Audio Plugins
 *     Copyright (C) 2020-2023 Pere Rafols Soler
 * 
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 * 
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **************************************************************************/

#ifndef IIR_FILTER_BASE
#define IIR_FILTER_BASE

#define _USE_MATH_DEFINES
#include <cmath>
#include "param_interpolator.h"

#define DENORMAL_TO_ZERO(x) if (fabs(x) < (1e-300)) x = 0.0; //Min float is 1.1754943e-38 (Min double is 2.23×10−308)

//Data type to share the IIR coeficients along multiple IIRfilter instances
typedef struct
{
  double b0, b1, b2, a1, a2; //Second Order coeficients  
}filterCoefs;


//Class to implement the IIR filter
class IIRfilter
{
public:
    IIRfilter(filterCoefs *f, double rate)
    {
        filter = f;
        flushBuffers();
        enable = enable_target = 0.0;
        InterK = INTERPOLATOR_CALC_K((float)rate);
    }
    
    ~IIRfilter()
    {
    }
    
    inline void run(double *inputSample, double *outputSample)
    {
        //Filter bypass interpolation
        if(enable != enable_target)
        {
            enable = (double)computeParamInterpolation((float)enable_target, (float)enable, InterK, true);
        }
        
        //w(n)=x(n)-a1*w(n-1)-a2*w(n-2)
        buf_0 = (*inputSample)-filter->a1*buf_1-filter->a2*buf_2;                   
            
        DENORMAL_TO_ZERO(buf->buf_0);
        //y(n)=bo*w(n)+b1*w(n-1)+b2*w(n-2)
        *outputSample = (filter->b0*buf_0 + filter->b1*buf_1+ filter->b2*buf_2) * enable + (*inputSample)*(1.0 - enable);

        buf_2 = buf_1;
        buf_1 = buf_0;
    }
    
    void setEnable(double Enabled)
    {
        enable_target = Enabled;
    }
    
    bool getEnabled()
    {
        return enable_target;
    }
    
    void flushBuffers()
    {
        buf_0 = 0.0;
        buf_1 = 0.0;
        buf_2 = 0.0;
    }
    
    void sampleRateChanged(double rate)
    {
        InterK = INTERPOLATOR_CALC_K((float)rate);
        flushBuffers();
    }

private:
    filterCoefs *filter; //Pointer to a shared filter coeficients struct
    double buf_0, buf_1, buf_2; //Filter buffers
    double enable, enable_target; //Filter bypass with interpolation (cross-fading)
    float InterK;
};

//define a name for all the implemented filter types
enum FilterType
{ 
    LPF1,   //First order low pass 
    LPF2,   //Second orfer low pass
    HPF1,   //First order high pass
    HPF2,   //Second horder high pass
    NOTCH,  //Biquad notch
    LSHELF, //Low shelfing
    HSHELF, //High shelfing
    PEAK    //Biquad peaking
};

//Class to implent the IIR coeficients calculation and interpolation
class IIRcoefs
{
public:
    IIRcoefs(double rate, FilterType ftype)
    {
     fs = rate;   
     gain = 0.0;
     freq = 0.0;
     q = 0.0;
     filType = ftype;
         
     filter.a1 = 0.0;
     filter.a2 = 0.0;
     filter.b0 = 0.0;
     filter.b1 = 0.0;
     filter.b2 = 0.0;
     
     InterK = INTERPOLATOR_CALC_K((float)rate);
     recompute_coefs = true;
    }
    
    ~IIRcoefs()
    {
    }
    
    void setGain(double Gain)
    {
        gain_target = Gain;
        recompute_coefs = true;
    }
    
    void setFreq(double Freq)
    {
        freq_target = Freq;
        recompute_coefs = true;
    }
    
    void setQ(double Q)
    {
        q_target = Q;
        recompute_coefs = true;
    }
    
    void setFilterType(FilterType ftype)
    {
        filType = ftype;
        recompute_coefs = true;
    }
    
    filterCoefs* getCoefsPointer()
    {
        return &filter;
    }
    
    void sampleRateChanged(double rate)
    {
        InterK = INTERPOLATOR_CALC_K((float)rate);
        fs = rate;
        recompute_coefs = true;
    }
    
    //Compute filter coeficients
    inline void calcCoefs()
    {   
        if(recompute_coefs)
        {
            double alpha, A, b0, b1, b2, a0, a1, a2, b1_0, b1_1, b1_2, a1_0, a1_1, a1_2;
            alpha = A = b0 = b1 = b2 = a0 = a1 = a2 = b1_0 = b1_1 = b1_2 = a1_0 = a1_1 = a1_2 = 1.0;
            
            //Param Interpolation    
            freq = computeParamInterpolation(freq_target, freq, InterK, param_interpolation);
            gain = computeParamInterpolation(gain_target, gain, InterK, param_interpolation);
            q = computeParamInterpolation(q_target, q, InterK, param_interpolation);
            
            if((gain == gain_target) && (freq == freq_target) && (q == q_target)) 
            {
                recompute_coefs = false;
            }
            
            double w0=2*M_PI*(freq/fs);
                
            switch(filType)
            {
                case FilterType::HPF1:
                    w0 = tanf(w0/2.0);
                    b0 = 1.0;
                    b1 = -1.0;
                    b2 = 0.0;
                    a0 = w0+1.0;
                    a1 = w0-1.0;
                    a2 = 0.0;
                break;

                case FilterType::HPF2:
                    alpha = sinf(w0)/(2*q);
                    b1_0 = b0 = (1 + cosf(w0))/2; //b0
                    b1_1 = b1 = -(1 + cosf(w0)); //b1
                    b1_2 = b2 = (1 + cosf(w0))/2; //b2
                    a1_0 = a0 = 1 + alpha; //a0
                    a1_1 = a1 = -2*cosf(w0); //a1
                    a1_2 = a2 = 1 - alpha; //a2
                break;

                case FilterType::LPF1: 
                    w0 = tanf(w0/2.0);
                    b0 = w0;
                    b1 = w0;
                    b2 = 0.0;
                    a0 = w0+1.0;
                    a1 = w0-1.0;
                    a2 = 0.0;
                break;
        
                case FilterType::LPF2:
                    alpha = sinf(w0)/(2*q);
                    b1_0 = b0 = (1 - cosf(w0))/2; //b0
                    b1_1 = b1 = 1 - cosf(w0); //b1
                    b1_2 = b2 = (1 - cosf(w0))/2; //b2
                    a1_0 = a0 = 1 + alpha; //a0
                    a1_1 = a1 = -2*cosf(w0); //a1
                    a1_2 = a2 = 1 - alpha; //a2
                break;

                case FilterType::LSHELF:
                    A = sqrtf((gain));
                    alpha =sinf(w0)/2 * (1/q);
                    b0 = A*((A+1)-(A-1)*cosf(w0)+2*sqrtf(A)*alpha); //b0
                    b1 = 2*A*((A-1)-(A+1)*cosf(w0)); //b1
                    b2 = A*((A+1)-(A-1)*cosf(w0)-2*sqrtf(A)*alpha); //b2
                    a0 = (A+1) + (A-1)*cosf(w0) + 2*sqrtf(A)*alpha; //a0
                    a1 = -2*((A-1) + (A+1)*cosf(w0)); //a1
                    a2 = (A+1) + (A-1)*cosf(w0) - 2*sqrtf(A)*alpha; //a2
                break;

                case FilterType::HSHELF:
                    A = sqrtf((gain));
                    alpha =sinf(w0)/2 * (1/q);
                    b0 = A*( (A+1) + (A-1)*cosf(w0) + 2*sqrtf(A)*alpha ); //b0
                    b1 = -2*A*( (A-1) + (A+1)*cosf(w0)); //b1
                    b2 = A*( (A+1) + (A-1)*cosf(w0) - 2*sqrtf(A)*alpha ); //b2
                    a0 = (A+1) - (A-1)*cosf(w0) + 2*sqrtf(A)*alpha; //a0
                    a1 = 2*( (A-1) - (A+1)*cosf(w0)); //a1
                    a2 = (A+1) - (A-1)*cosf(w0) - 2*sqrtf(A)*alpha; //a2
                break;
                                
                case FilterType::NOTCH:
                    alpha = sinf(w0)/(2*q);
                    b0 =  1; //b0
                    b1 = -2*cosf(w0); //b1
                    b2 =  1; //b2
                    a0 =  1 + alpha; //a0
                    a1 = -2*cosf(w0); //a1
                    a2 =  1 - alpha; //a2
                break;

                case FilterType::PEAK:
                    A = sqrtf(gain);
                    double A2 = A*A;
                    double PI2 = M_PI*M_PI;
                    double Q2 = q*q;
                    double w02 = w0 * w0;
                    double w02_PI22 = (w02 - PI2)*(w02 - PI2);
                    
                    //Equivalent analog filter and analog gains
                    double G1 = sqrtf((w02_PI22 + (A2*w02*PI2)/Q2)/(w02_PI22 + (w02*PI2)/(Q2*A2)));
                    double GB = sqrt(G1*gain);
                    double GB2 = GB * GB;
                    double G2 = gain * gain;
                    double G12 = G1 * G1;
                    
                    //Digital filter
                    double F   = fabs(G2  - GB2);// + 0.00000001f; ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0 
                    double G00 = fabs(G2  - 1.0);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double F00 = fabs(GB2 - 1.0);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double G01 = fabs(G2  - G1);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double G11 = fabs(G2  - G12);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double F01 = fabs(GB2 - G1);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double F11 = fabs(GB2 - G12);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
                    double W2 = sqrtf(G11 / G00) * tanf(w0/2.0) * tanf(w0/2.0);

                    //Bandwidth condition
                    double Aw = (w0/(A*q))*sqrtf((GB2-A2 * A2)/(1.0 - GB2)); //Analog filter bandwidth at GB
                    double DW = (1.0 + sqrtf(F00 / F11) * W2) * tanf(Aw/2.0); //Prewarped digital bandwidth
                    
                    //printf("G1=%f Aw=%f DW=%f F11=%f GB2=%f G12=%f\r\n",G1,Aw,DW,F11,GB2,G12);
                    
                    //Digital coefs
                    double C = F11 * DW * DW - 2.0 * W2 * (F01 - sqrtf(F00 * F11));
                    double D = 2.0 * W2 * (G01 - sqrtf(G00 * G11));
                    double A = sqrtf((C + D) / F);
                    double B = sqrtf((G2 * C + GB2 * D) / F);
                    
                    //printf("A=%f B=%f C=%f D=%f W2=%f F=%f G2=%f GB2=%f\r\n", A, B, C, D, W2, F, G2, GB2 );
                    
                    if( gain > 1.01 || gain < 0.98 )
                    {
                        b0 = G1 + W2 + B;
                        b1 =  -2.0*(G1 - W2);
                        b2 = G1 - B + W2;
                        a0 = 1.0 + W2 + A;
                        a1 = -2.0*(1.0 - W2);
                        a2 = 1.0 + W2 - A;
                    }
                    else
                    {
                        b0 = 1.0;
                        b1 = 0.0;
                        b2 = 0.0;
                        a0 = 1.0;
                        a1 = 0.0;
                        a2 = 0.0;
                    }
                break;
            } //End of switch

            //Normalice coeficients to a0=1 
            filter.b0 = (b0/a0); //b0
            filter.b1 = (b1/a0); //b1
            filter.b2 = (b2/a0); //b2
            filter.a1 = (a1/a0); //a1
            filter.a2 = (a2/a0); //a2
            
            //Print coefs
            //printf("Coefs b0=%f b1=%f b2=%f a1=%f a2=%f\r\n",filter->b0,filter->b1,filter->b2,filter->a1,filter->a2);
            //printf("Gain = %f Freq = %f Q = %f\r\n", filter->gain, filter->freq, filter->q);
        }
    }

    
private:
    double fs; //Store the sample rate
    double gain, freq, q;
    double gain_target, freq_target, q_target;
    bool param_interpolation; //True when param interpolation must be enabled
    float InterK;
    FilterType filType;
    filterCoefs filter;
    bool recompute_coefs; //bool to signal when coeficients must be recomputed
};

#endif
