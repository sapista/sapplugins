// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

FreqGrid::FreqGrid()    
{    
    recompute_pixel_mappings();
}

void FreqGrid::paint (juce::Graphics& g) 
{
    g.drawImageAt (gridCache, 0, 0);
}

void FreqGrid::resized()
{
    triggerAsyncUpdate();
}

void FreqGrid::handleAsyncUpdate()
{
    renderGridToCache();
    repaint();
}

float FreqGrid::getFreqMin()
{
    return freq_min;
}

float FreqGrid::getFreqMax()
{
    return freq_max;
}

float FreqGrid::getRangedB()
{
    return dB_range;
}

void FreqGrid::recompute_pixel_mappings()
{
    pixel2dB_m = -dB_range/(static_cast<float> (getHeight()) - 2.0f*plot_margin);
    pixel2dB_n = (dB_range*0.50f) - plot_margin*pixel2dB_m;
    dB2pixel_m = 1.0f/pixel2dB_m;
    dB2pixel_n = plot_margin - dB_range*dB2pixel_m*0.5f;
    pixel2freq_m = std::log2(freq_max/freq_min)/(static_cast<float> (getWidth()) - 2.0f*plot_margin);
    pixel2freq_n = std::log2(freq_min) - plot_margin*pixel2freq_m;
    freq2pixel_m = 1.0f/pixel2freq_m;
    freq2pixel_n = plot_margin - freq2pixel_m*std::log2(freq_min); 
}

float FreqGrid::map_Pixel2dB(float pixel)
{
    return pixel*pixel2dB_m + pixel2dB_n;   
}

float FreqGrid::map_dB2Pixel(float dB)
{   
    return dB*dB2pixel_m + dB2pixel_n;   
}

float FreqGrid::map_Pixel2Hz(float pixel)
{
    return std::pow( 2.0f, pixel*pixel2freq_m + pixel2freq_n);   
}

float FreqGrid::map_Hz2Pixel(float Hz)
{
    return std::log2(Hz)*freq2pixel_m+freq2pixel_n;   
}

void FreqGrid::renderGridToCache()
{
    recompute_pixel_mappings();
    
    gridCache = juce::Image (juce::Image::ARGB, getWidth(), getHeight(), true);
    juce::Graphics g (gridCache);
    
    g.fillAll (juce::Colours::black); //TODO fill with a nice gradient and also limit the paint to rounded corner rectangle

    g.setColour(sap::ColourGrid);

    // Define the area (x, y, width, height)
    auto area = getLocalBounds().toFloat().reduced (plot_margin);
    
    // To draw an outline:
    g.drawRoundedRectangle (area, 5.0f, 2.0f); // 2.0f is line thickness
    
    auto bounds = getLocalBounds().toFloat();
    
    // --- Draw dB Grid (Horizontal Lines) ---
    int dBLine_step = (static_cast<int>(dB_range)) / static_cast<int>(numdBLines);
    const float mindB_val =  ((static_cast<int>(-0.5f*dB_range + 2.0f)) / dBLine_step ) * dBLine_step;
    float plotdB = mindB_val;
    while(plotdB <= 0.5f*dB_range - 2.0f)
    {
        float y = map_dB2Pixel(plotdB);
        g.drawHorizontalLine (static_cast<int>(y), bounds.getX()+plot_margin, bounds.getRight()-plot_margin);
        g.drawText (juce::String (plotdB, 0), 0, y - 10, plot_margin-4, 20, juce::Justification::right);
        plotdB += dBLine_step;
    }
    
    // --- Draw Frequency Grid (Vertical Lines) ---
    const std::vector<float> freqsToDrawVerticalLine ={20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0,
                        100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0,
                        1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0, 9000.0,
                        10000.0, 20000.0};
    
    for (auto f : freqsToDrawVerticalLine)
    {
        float x = map_Hz2Pixel(f);
        if( x >= freq_min && x <= freq_max)
        {
            g.drawVerticalLine (static_cast<int>(x), bounds.getY()+plot_margin, bounds.getBottom()-plot_margin);
        }
    }
    
    const std::vector<float> freqsToDrawLabel ={20.0, 50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0, 5000.0, 20000.0};
    for (auto f : freqsToDrawLabel)
    {
        float x = map_Hz2Pixel(f);
        if( x >= freq_min && x <= freq_max)
        {
            g.drawText (f >= 1000 ? juce::String (f/1000) + "k" : juce::String (f), 
                    x - 20, bounds.getBottom() - plot_margin + 4, 40, 20, juce::Justification::centredTop);
        }
    }
}
