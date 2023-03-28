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


#include <iostream>

#include "DistrhoPlugin.hpp"

START_NAMESPACE_DISTRHO

// -----------------------------------------------------------------------------------------------------------

/**
 Parametric equalizer with linear phase feature based on time-reversal approach.
 */
//TODO this starts as the mono version and I'll derive the code to make it stereo with mid-side processing when tested. 
//TODO Ill be using pre-processor and Cmake to share the code between mono and stereo versions
class LPEQPlugin : public Plugin
{
public:
    LPEQPlugin()
        : Plugin(4, 0, 0) // 4 parameters, 0 programs, 0 states
    {
       //TODO iniit parameters, right now I'm testing a single band EQ, so four params: Latency, Gain, freq, Q
        
        host_requested_latency_seconds = -1.0f; //Starting with an impossible value to force host to set the latency using the corresponding plugin port
        segment_length = 0;
        filtering_backwards = true;
        nextSegmentBuffer = nullptr;
        currentSegmentBuffer = nullptr;
        output_segment_buffer = nullptr;
    }
    
    ~LPEQPlugin()
    {
        //TODO call the method to erase buffers
    }

protected:
   /* --------------------------------------------------------------------------------------------------------
    * Information */

   /**
      Get the plugin label.
      A plugin label follows the same rules as Parameter::symbol, with the exception that it can start with numbers.
    */
    const char* getLabel() const override
    {
        return "LPEQ-Mono"; //TODO return stereo in case of stereo! Use C pre-processor!
    }

   /**
      Get an extensive comment/description about the plugin.
    */
    const char* getDescription() const override
    {
        return "Linear-Phase equalizer"; //TODO improve me!
    }

   /**
      Get the plugin author/maker.
    */
    const char* getMaker() const override
    {
        return "SapAudio";
    }

   /**
      Get the plugin homepage.
    */
    const char* getHomePage() const override
    {
        return "http://sapaudio.org/plugins/lpeq";
    }

   /**
      Get the plugin license name (a single line of text).
      For commercial plugins this should return some short copyright information.
    */
    const char* getLicense() const override
    {
        return "ISC";  //TODO how to set to GPL3
    }

   /**
      Get the plugin version, in hexadecimal.
    */
    uint32_t getVersion() const override
    {
        return d_version(1, 0, 0);
    }

   /**
      Get the plugin unique Id.
      This value is used by LADSPA, DSSI and VST plugin formats.
    */
    int64_t getUniqueId() const override
    {
        return d_cconst('L', 'P', 'Q', 'M'); //TODO for the stereo plugin must be ('L', 'P', 'Q', 'S')
    }

   /* --------------------------------------------------------------------------------------------------------
    * Init */

   /**
      Initialize the audio port @a index.@n
      This function will be called once, shortly after the plugin is created.
    */
    void initAudioPort(bool input, uint32_t index, AudioPort& port) override
    {
        // treat meter audio ports as stereo
        port.groupId = kPortGroupStereo;

        // everything else is as default
        Plugin::initAudioPort(input, index, port);
    }

   /**
      Initialize the parameter @a index.
      This function will be called once, shortly after the plugin is created.
    */
    void initParameter(uint32_t index, Parameter& parameter) override
    {
        
        switch (index)
        {
        case 0:
            parameter.name = "Latency";
            parameter.unit   = "s";
            parameter.ranges.def = 2.0f;
            parameter.ranges.min = 0.500f;
            parameter.ranges.max = 6.0f;
            break;
            
        case 1:
            parameter.hints = kParameterIsAutomatable;
            parameter.name = "Band1-Gain";
            parameter.unit   = "dB";
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -20.0f;
            parameter.ranges.max = 20.0f;
            break;
            
        case 2:
            parameter.hints = kParameterIsAutomatable;
            parameter.name = "Band1-Freq";
            parameter.unit   = "Hz";
            parameter.ranges.def = 500.0f;
            parameter.ranges.min = 20.0f;
            parameter.ranges.max = 20000.0f;
            break;
            
        case 3:
            parameter.hints = kParameterIsAutomatable;
            parameter.name = "Band1-Q";
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 0.1f;
            parameter.ranges.max = 10.0f;
            break;
    
        }

       /**
          Our parameter names are valid symbols except for "-".
        */
        parameter.symbol = parameter.name;
        parameter.symbol.replace('-', '_');
    }

   
   /* --------------------------------------------------------------------------------------------------------
    * Internal data */

   /**
      Get the current value of a parameter.
    */
    float getParameterValue(uint32_t index) const override
    {
        switch (index)
        {
        case 0: return (3.0 * (double)segment_length) / getSampleRate();     
        case 1: return 0.0f; //TODO get from filter class
        case 2: return 0.0f; //TODO get from filter class
        case 3: return 0.0f; //TODO get from filter class
        }
        
        return -1.0f; //Should never happen
    }

   /**
      Change a parameter value.
    */
    void setParameterValue(uint32_t index, float value) override
    {
        switch (index)
        {
            case 0: 
                if( host_requested_latency_seconds != value)
                {
                    bufferDestructor(); //Ensure we start with fresh buffers
                    host_requested_latency_seconds = value;                        
                    segment_length = (unsigned int) floor( (double)host_requested_latency_seconds * getSampleRate() / 3.0 );
                    bufferConstructor();
                }
                
                break;
                
            case 1:
                //TODO
                break;
                
            case 2:
                //TODO
                break;
                
            case 3:
                //TODO
                break;
        }
    }

   
   /* --------------------------------------------------------------------------------------------------------
    * Process */
 
   
   /**
      Run/process function for plugins without MIDI input.
    */
    void run(const float** inputs, float** outputs, uint32_t frames) override
    {   
        if(segment_length == 0)
        {
            std::cout<<"Buffers not initialized!"<<std::endl;
            return; //Waiting for buffer Initialize
        }
        
        //TODO import parameter interpolation from EQ10Q
        
        for(int i = 0; i < 5*frames; i++)
        {   
            //TODO DEBGUG Prints
            //std::cout<<"i = "<<i<<std::endl;
            //std::cout<<"i_sample = "<<i_sample<<" of "<<segment_length<<" filtering_backwards = "<< filtering_backwards << std::endl;
            //std::cout<<"i_outSegmentBuffer_read = "<<i_outSegmentBuffer_read<<std::endl;
            //std::cout<<"i_outSegmentBuffer_write = "<<i_outSegmentBuffer_write<<std::endl;
            
            //One every five iterations the next segment buffer is filled
            if( i % 5 == 0 )
            { 
            
                //TODO DEBGUG Prints
                //std::cout<<"i/5 = "<<i/5<<" of " << frames << std::endl;
            
                
                //Read input buffers and store it to next processing buffer
                //Note that the segment buffer is two times the legth of a segment.
                //This allows to store the pre-ringing, and the body.
                //So, here I'm storing the input buffer at the body part of the segment buffer
                nextSegmentBuffer[i_nextSegment + segment_length] = inputs[0][i/5]; //TODO implement also in stereo
                i_nextSegment++;
                i_nextSegment = (i_nextSegment >= segment_length) ? 0 : i_nextSegment;
                                
                //Write to the output buffer
                outputs[0][i/5] = output_segment_buffer[i_outSegmentBuffer_read]; //TODO do it in stereo too!
                i_outSegmentBuffer_read++;
                i_outSegmentBuffer_read = (i_outSegmentBuffer_read >= (3*segment_length)) ? 0 : i_outSegmentBuffer_read; //Output circular buffer                 
            
            }
                    
            if(filtering_backwards)
            {               
                //Backward filter is run first and overwrites the input buffer
                currentSegmentBuffer[i_sample] = runIIR(currentSegmentBuffer[i_sample]); 
                if(i_sample == 0 )
                {
                    //Backward filtering complete, next is forward
                    i_sample = 0;
                    filtering_backwards = false;
                    reset_IIR_filters(); //Call a function to set to zero all dealy elements of the IIR filters 
                }
                else
                {
                    i_sample--;
                }
            }
            else
            {
                //Forward filtering
                if(i_sample < 2*segment_length)
                {
                    //Processing the pre-ringing and the body, thus overlap-add is taking place
                    output_segment_buffer[i_outSegmentBuffer_write] += runIIR(currentSegmentBuffer[i_sample]);
                }
                else
                {
                    //Processing the post-ringing, so the output buffer must be over-written
                    output_segment_buffer[i_outSegmentBuffer_write] = runIIR(0.0);
                }
                        
                i_sample++;
                i_outSegmentBuffer_write++;
                i_outSegmentBuffer_write = (i_outSegmentBuffer_write >= (3*segment_length)) ? 0 : i_outSegmentBuffer_write; //Output circular buffer
                if(i_sample == 3*segment_length)
                {
                    //Prepare the processing for the next segment
                    i_sample = 2*segment_length - 1; //IIR filtering will start backwards from the end of body toward the pre-ringing
                    filtering_backwards = true; //set the processing time reversed for the next segment
                    reset_IIR_filters(); //Call a function to set to zero all delay elements of the IIR filters
                    
                    //Invert input buffers
                    double *aux_ptr = currentSegmentBuffer;
                    currentSegmentBuffer = nextSegmentBuffer;
                    nextSegmentBuffer = aux_ptr;
                    
                    //Set to zero the pre-ringing area of the next input buffer
                    for(int i = 0; i < segment_length; i++)
                    {
                        nextSegmentBuffer[i] = 0.0;
                    }
                    
                    //Set next output buffer write positon, this is one segment length delayed
                    i_outSegmentBuffer_write += segment_length;
                    i_outSegmentBuffer_write = (i_outSegmentBuffer_write >= (3*segment_length)) ? 0 : i_outSegmentBuffer_write; //Output circular buffer
                }
                
                                
            }
                
        }
    }

    // -------------------------------------------------------------------------------------------------------
    
    
   /* --------------------------------------------------------------------------------------------------------
    * Callbacks (optional) */

   /**
      Optional callback to inform the plugin about a sample rate change.
      This function will only be called when the plugin is deactivated.
    */
    void sampleRateChanged(double newSampleRate) override
    {
        bufferDestructor();
        segment_length = floor( host_requested_latency_seconds * newSampleRate / 3.0 );
        bufferConstructor();
    }

    // -------------------------------------------------------------------------------------------------------


private:
    
    /**
        Construct the internal processing buffers.
    */
    void bufferConstructor()
    {
        //Prepare the input buffers used for storing the input samples and then performing the IIR time reversed filtering.
        //The length of theses buffers are two times the segment_length. Used as follows:
        // first segment_length is used as zero-padding to latter store the preringing
        // the second segment_length is used to store the signal body
        nextSegmentBuffer = new double[2*segment_length];
        currentSegmentBuffer = new double[2*segment_length];
        for(int i = 0; i < (2*segment_length); i++)
        {
            nextSegmentBuffer[i] = 0.0;
            currentSegmentBuffer[i] = 0.0;
        }
        i_nextSegment = 0; //index of the loading segment buffer
        
        //The circular output buffer
        output_segment_buffer = new double[3*segment_length];
        i_outSegmentBuffer_read = 2*segment_length; //Index for reading the output buffer, one segment delayed the read 
        i_outSegmentBuffer_write = 0; //Index for writting the output buffer
        for(int i = 0; i < (3*segment_length); i++)
        {
            output_segment_buffer[i] = 0.0; //init Output buffer to avoid traching the output at the startup
        }
        
        //Init filters
        i_sample = 2*segment_length - 1; //IIR filtering will start backwards from the end of body toward the pre-ringing
        filtering_backwards = true; 
        reset_IIR_filters(); //Call a function to set to zero all delay elements of the IIR filters
        
        //TODO hte DISTRHO doc says this: This function should only be called in the constructor, activate() and run().  But in the Latency example they are calling it here!
        //TODO enable me!
        setLatency(3*segment_length);
    }
    
    /**
     Destroy the internal buffers and free memory
     */ 
    void bufferDestructor()
    {
        delete[] nextSegmentBuffer;
        delete[] currentSegmentBuffer;
        delete[] output_segment_buffer;
        nextSegmentBuffer = nullptr;
        currentSegmentBuffer = nullptr;
        output_segment_buffer = nullptr;
        segment_length = 0;
        i_nextSegment = 0;
        i_outSegmentBuffer_read = 0;
        i_outSegmentBuffer_write = 0;
        i_sample = 0;
    }
    
    /**
     Reset the internal buffers of all the IIR filters
     */ 
    void reset_IIR_filters()
    {
        //TODO implement me by calling the buffer flush method of all filters used 
    }
    
    /**
     Execute the IIR processing for a single sample going trought all instantianed filter objects
     */ 
    inline double runIIR(double x)
    {
        //TODO execute the proc here,
        //TODO think how to extend this to stereo + mid/side mode... should I have 4 functions like this (one for each auio stream in the processing)? 
        return x;
    }
      
   /**
    Plugin latency 
    host_requested_latency_seconds: is the requested latency by the host using a plugin port (in seconds)
    latency_samples: is the latency calculated in samples using 3*floor( host_requested_latency_seconds * getSampleRate() )
    */
   float host_requested_latency_seconds;
   unsigned int segment_length;
   
   
   double *nextSegmentBuffer; //Pointer to the next segment of audio data to process
   double *currentSegmentBuffer; //Pointer to the current segment buffer being processed
   unsigned int i_nextSegment; //Index for loading data in the nextSegmentBuffer
   
   double *output_segment_buffer; //Buffer to store the processed data and apply the overlap-add method to it
   unsigned int i_outSegmentBuffer_read; //Index to read output_segment_buffer and place the data to the plugin output void initPortGroup(uint32_t groupId, DISTRHO::PortGroup & portGroup) override;
   unsigned int i_outSegmentBuffer_write; //Index to write data in the processing output buffer
   unsigned int i_sample; //Index of the current sample processed
   bool filtering_backwards; //A boolean signalling whether the processing is in time reverse
   
   //TODO instatntiate filters from the imported filter class and set the parameters there!

   /**
      Set our plugin class as non-copyable and add a leak detector just in case.
    */
    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(LPEQPlugin)
};

/* ------------------------------------------------------------------------------------------------------------
 * Plugin entry point, called by DPF to create a new plugin instance. */

Plugin* createPlugin()
{
    return new LPEQPlugin();
}

// -----------------------------------------------------------------------------------------------------------

END_NAMESPACE_DISTRHO
