// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once
#include <vector>
#include <memory>

class FreqGrid : public juce::Component, public juce::AsyncUpdater
{
public:
    FreqGrid();
    void paint (juce::Graphics& g) override;
    void resized() override;
    void handleAsyncUpdate() override;
    
    //Data accessors
    float getFreqMin();
    float getFreqMax();
    float getRangedB();
    
    // Signals
    //TODO
    
protected:
    
    //Methods
    void recompute_pixel_mappings();
    float map_Pixel2dB(float pixel);
    float map_dB2Pixel(float dB);
    float map_Pixel2Hz(float pixel);
    float map_Hz2Pixel(float Hz);

private:    
    
    static constexpr float plot_margin = 30.0f;
    static constexpr uint numdBLines = 12;    
    
    static constexpr float default_dB_range = 32.0f;
    static constexpr float default_freq_min = 10.0f;
    static constexpr float default_freq_max = 22.05e3f;
    
    //These ranges will be modified during zoom/pan on the freq widget
    float dB_range = default_dB_range;
    float freq_min = default_freq_min;
    float freq_max = default_freq_max;
    
    //Precomputed on rezise m and n for freq and dB mapping
    float pixel2dB_m; 
    float pixel2dB_n; 
    float dB2pixel_m;
    float dB2pixel_n;
    float pixel2freq_m;
    float pixel2freq_n;
    float freq2pixel_m;
    float freq2pixel_n;
    
    juce::Image gridCache;
    void renderGridToCache();
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FreqGrid)
};

