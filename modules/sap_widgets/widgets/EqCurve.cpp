// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

EqCurve::EqCurve(const uint band_count):
    bandCount(band_count)
{
    for (uint i = 0; i < bandCount; ++i)
    {
        auto newBand = std::make_unique<sap::EqBandControl>(i);
        addAndMakeVisible (newBand.get());
        EqBands.push_back (std::move (newBand));
        
        auto* band = EqBands[i].get();
        band->onMouseDrag = [this, band] (float newPosX, float newPosY)
        {
            int newX = static_cast<int>(newPosX); 
            int newY = static_cast<int>(newPosY); 

            
            // Limit movement to curve bounds
            newX = juce::jlimit(static_cast<int>(map_Hz2Pixel(getFreqMin())), 
                                static_cast<int>(map_Hz2Pixel(getFreqMax())),
                                newX);
           
            newY = juce::jlimit(static_cast<int>(map_dB2Pixel(0.5f*getRangedB())),
                                static_cast<int>(map_dB2Pixel(-0.5f*getRangedB())), 
                                newY);

            // Move the entire BandControl
            band->setCentrePosition(newX, newY);
            
            //Get Hz from X coord
            band->setFreq( map_Pixel2Hz(newX));
            //TODO emit freq signal
            
            //Get dB from Y coord
            band->setGain( map_Pixel2dB(newY));
            //TODO emit Gain signal
        };
        
        band->onGainChanged = [this,band] (float x,  juce::Point<float> offset)
        {
            // Move the entire BandControl
            band->setCentrePosition(band->getBounds().getCentreX(), (int)map_dB2Pixel(x));
            
            //TELEPORT cursor
            auto parentSpaceCenter = band->getBounds().getCentre().toFloat();
            auto globalCenter = localPointToGlobal (parentSpaceCenter);
            globalCenter.x += offset.x;
            globalCenter.y += offset.y;
            juce::Desktop::getInstance().setMousePosition (globalCenter.roundToInt());
            
            DBG("Gain Changed: " << x);
            //TODO emit Gain signal
        };
        
        band->onFreqChanged = [this,band] (float x,  juce::Point<float> offset)
        {
            // Move the entire BandControl
            band->setCentrePosition((int)map_Hz2Pixel(x), band->getBounds().getCentreY());
            
            //TELEPORT cursor
            auto parentSpaceCenter = band->getBounds().getCentre().toFloat();
            auto globalCenter = localPointToGlobal (parentSpaceCenter);
            globalCenter.x += offset.x;
            globalCenter.y += offset.y;
            juce::Desktop::getInstance().setMousePosition (globalCenter.roundToInt());
            
            DBG("Freq Changed: " << x);
            //TODO emit Freq signal
        };
        
        band->onQChanged = [this,band] (float x)
        {
            DBG("Q Changed: " << x);
            //TODO emit Q signal
        };
    }
}

void EqCurve::paint (juce::Graphics& g) 
{
    FreqGrid::paint(g);
}

void EqCurve::resized()
{
    FreqGrid::resized();
}
