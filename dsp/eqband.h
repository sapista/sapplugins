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

#ifndef EQ_BAND
#define EQ_BAND

#include "iirfilter.h"

//Base EQ Band class without any filter. Intended to be derivated to create specific EQ bands
class BaseEqBand
{
public:
    BaseEqBand(double rate, FilterType ftype, unsigned int numberOfFilters)
    {
        coefsObj = new IIRcoefs(rate, ftype);
        filterCount = numberOfFilters;
        filterObjs = new IIRfilter*[filterCount];
        for(int i = 0; i < filterCount; i++)
        {
            filterObjs[i] = new IIRfilter(coefsObj->getCoefsPointer(), rate);
        }
    }
    
    ~BaseEqBand()
    {
        for(int i = 0; i < filterCount; i++)
        {
            delete filterObjs[i];
        }
        delete[] filterObjs;
        delete coefsObj;
    }
    
    void setGain(double Gain)
    {
        coefsObj->setGain(Gain);
    }
    
    void setFreq(double Freq)
    {
        coefsObj->setFreq(Freq);
    }
    
    void setQ(double Q)
    {
        coefsObj->setQ(Q);
    }
    
    void setFilterType(FilterType ftype)
    {
        coefsObj->setFilterType(ftype);
    }
    
    void flushBuffers()
    {
        for(int i = 0; i < filterCount; i++)
        {
            filterObjs[i] -> flushBuffers();
        }
    }
    
    void sampleRateChanged(double rate)
    {
        coefsObj->sampleRateChanged(rate);
        for(int i = 0; i < filterCount; i++)
        {
            filterObjs[i] -> sampleRateChanged(rate);
        }
    }
    
protected:
    IIRcoefs *coefsObj; //A single coeficient objects is shared for all filters in the EQ band
    IIRfilter **filterObjs; //Array of filter objects
    unsigned int filterCount; //Track the total number of filters
    
    void addFilter();
};

//A simple band with a single filter
class SingleFilterEqBand : public BaseEqBand
{
public:
    SingleFilterEqBand(double rate, FilterType ftype) : BaseEqBand(rate, ftype, 1)
    {
    }
        
    void setEnable(double Enabled)
    {
        filterObjs[0]->setEnable(Enabled);
    }
    
    inline void run(double *inputSample, double *outputSample)
    {
        coefsObj->calcCoefs();
        filterObjs[0]->run(inputSample, outputSample);
    }
    
};

//A simple stereo band with two filters
class SimpleStereoEqBand : public BaseEqBand
{
public:
    SimpleStereoEqBand(double rate, FilterType ftype) : BaseEqBand(rate, ftype, 2)
    {
        //Set mapping to facilitate development
        filterL = filterObjs[0];
        filterR = filterObjs[1];
    }
    
    void setEnableL(bool Enabled)
    {
        filterL->setEnable((double)Enabled);
    }
    
    void setEnableR(bool Enabled)
    {
        filterR->setEnable((double)Enabled);
    }
    
    inline void run(double *inputSampleL, double *inputSampleR, double *outputSampleL, double *outputSampleR)
    {
        coefsObj->calcCoefs();
        filterL->run(inputSampleL, outputSampleL);
        filterR->run(inputSampleR, outputSampleR);
    }

private:
    IIRfilter *filterL, *filterR;
    
};

//Band for linear phase mono EQ
class LinearPhaseMonoEqBand : public BaseEqBand
{
public:
    LinearPhaseMonoEqBand(double rate, FilterType ftype) : BaseEqBand(rate, ftype, 3)
    {
        //Set mapping to facilitate development
        HB = filterObjs[0];
        HF1 = filterObjs[1];
        HF2 = filterObjs[1];
        
        linear_phase_mode = false;
        enable = false;
    }
    
    void setEnable(bool Enabled)
    {
        //Start by disabling all
        HB->setEnable(false);
        HF1->setEnable(false);
        HF2->setEnable(false);
        
        enable = Enabled;
        if(enable)
        {
            if(linear_phase_mode)
            {
                HB->setEnable(true);
                HF1->setEnable(true);
            }
            else
            {
                HF1->setEnable(true);
                HF2->setEnable(true);
            }
        }
        
    }
    
        //Set to true to activate the linear phase mode, set to false for minimum phase mode
    void setLinearPhase(bool bLinearPhaseMode) 
    {
        linear_phase_mode = bLinearPhaseMode;
        setEnable(enable);
    }
    
    void runBackward(double *inputSample, double *outputSample)
    {
        HB->run(inputSample, outputSample ); 
    }
    
    void runForward(double *inputSample, double *outputSample)
    {
       HF1->run(inputSample, outputSample );
       HF2->run(outputSample, outputSample ); 
       coefsObj->calcCoefs(); //Coefs computed always in the last step to be the same for both forward and backward
    }
    
private:
    IIRfilter *HB; //backward filter (for linear-phase)
    IIRfilter *HF1; //forward filter 1 (always used)
    IIRfilter *HF2; //forward filter 2 (for minimum-phase)
    bool linear_phase_mode;
    bool enable; //Band bypass control
};

//Band for linear phase stereo EQ with Mid/Side mode
class LinearPhaseStereoEqBand : public BaseEqBand
{
public:
    
    enum BandMode
    {
        STEREO_LINK,
        L_CHANNEL,
        R_CHANNEL,
        M_CHANNEL,
        S_CHANNEL
    };
    
    LinearPhaseStereoEqBand(double rate, FilterType ftype) : BaseEqBand(rate, ftype, 12)
    {
        //Set filter mappings
        HBL = filterObjs[0];
        HBR = filterObjs[1];
        HBM = filterObjs[2];
        HBS = filterObjs[3];
        HFL1 = filterObjs[4];
        HFL2 = filterObjs[5];
        HFR1 = filterObjs[6];
        HFR2 = filterObjs[7];
        HFM1 = filterObjs[8];
        HFM2 = filterObjs[9];
        HFS1 = filterObjs[10];
        HFS1 = filterObjs[11];
        
        linear_phase_mode = false;
        enable = false;
        mode = BandMode::STEREO_LINK;
        setEnable(enable);
    }
    
    void setEnable(bool Enabled)
    {
        //Start by disabling all
        HBL->setEnable(false);
        HBR->setEnable(false);
        HBM->setEnable(false);
        HBS->setEnable(false);
        HFL1->setEnable(false);
        HFL2->setEnable(false);
        HFR1->setEnable(false);
        HFR2->setEnable(false);
        HFM1->setEnable(false);
        HFM2->setEnable(false);
        HFS1->setEnable(false);
        HFS2->setEnable(false);
        
        enable = Enabled;
        if(enable)
        {
            if(linear_phase_mode)
            {
                //Linear-Phase processing
                switch(mode)
                {
                    case BandMode::L_CHANNEL:
                        HBL->setEnable(true);
                        HFL1->setEnable(true);
                    break;
                    
                    case BandMode::R_CHANNEL:
                        HBR->setEnable(true);
                        HFR1->setEnable(true);
                    break;
                    
                    case BandMode::M_CHANNEL:
                        HBM->setEnable(true);
                        HFM1->setEnable(true);
                    break;
                    
                    case BandMode::S_CHANNEL:
                        HBS->setEnable(true);
                        HFS1->setEnable(true);
                    break;
                    
                    case BandMode::STEREO_LINK:
                        HBL->setEnable(true);
                        HFL1->setEnable(true);
                        HBR->setEnable(true);
                        HFR1->setEnable(true);
                    break;
                }
            }
            else
            {
                //Minimum-Phase processing
                switch(mode)
                {
                    case BandMode::L_CHANNEL:
                        HFL1->setEnable(true);
                        HFL2->setEnable(true);
                    break;
                    
                    case BandMode::R_CHANNEL:
                        HFR1->setEnable(true);
                        HFR2->setEnable(true);
                    break;
                    
                    case BandMode::M_CHANNEL:
                        HFM1->setEnable(true);
                        HFM2->setEnable(true);
                    break;
                    
                    case BandMode::S_CHANNEL:
                        HFS1->setEnable(true);
                        HFS2->setEnable(true);
                    break;
                    
                    case BandMode::STEREO_LINK:
                        HFL1->setEnable(true);
                        HFL2->setEnable(true);
                        HFR1->setEnable(true);
                        HFR2->setEnable(true);
                    break;
                }
            }
        }
    }
    
    //Configure the band mode
    void setMode(BandMode Mode)
    {
        mode = Mode;
        setEnable(enable);
    }
    
    //Set to true to activate the linear phase mode, set to false for minimum phase mode
    void setLinearPhase(bool bLinearPhaseMode) 
    {
        linear_phase_mode = bLinearPhaseMode;
        setEnable(enable);
    }
    
    void runBackwardLR(double *inputSampleL, double *inputSampleR, double *outputSampleL, double *outputSampleR)
    {
        HBL->run(inputSampleL, outputSampleL);
        HBR->run(inputSampleR, outputSampleR);
    }
    
    void runBackwardMS(double *inputSampleM, double *inputSampleS, double *outputSampleM, double *outputSampleS)
    {
        HBM->run(inputSampleM, outputSampleM);
        HBS->run(inputSampleS, outputSampleS);
    }
    
    void runForwardLR(double *inputSampleL, double *inputSampleR, double *outputSampleL, double *outputSampleR)
    {
        HFL1->run(inputSampleL, outputSampleL);
        HFL2->run(outputSampleL, outputSampleL);
        HFR1->run(inputSampleR, outputSampleR);
        HFR2->run(outputSampleR, outputSampleR);
    }
    
    void runForwardMS(double *inputSampleM, double *inputSampleS, double *outputSampleM, double *outputSampleS)
    {
        HFM1->run(inputSampleM, outputSampleM);
        HFM2->run(outputSampleM, outputSampleM);
        HFS1->run(inputSampleS, outputSampleS);
        HFS2->run(outputSampleS, outputSampleS);
    }
    
    void calcCoefs()
    {
       coefsObj->calcCoefs(); 
    }
    
private:
    IIRfilter *HBL, *HBR; //Filters for time reversal L and R channels;
    IIRfilter *HBM, *HBS; //Filters for time reversal M and S channels;
    IIRfilter *HFL1, *HFR1, *HFL2, *HFR2; //Filters for time forward L and R channels;
    IIRfilter *HFM1, *HFS1, *HFM2, *HFS2; //Filters for time forward L and R channels;
    bool linear_phase_mode;
    bool enable; //Band bypass control
    BandMode mode;
};

#endif
