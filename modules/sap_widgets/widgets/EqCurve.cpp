// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

using namespace sap;

EqCurve::EqCurve(const std::vector<BandData>& bands)
{
    for (const auto& data : bands)
    {
        auto newBand = std::make_unique<sap::EqBandControl>(data);
        addAndMakeVisible (newBand.get());
        EqBands.push_back (std::move (newBand));
        
        auto* band = EqBands.back().get();        
        band->onMouseDrag = [this, band] (float newPosX, float newPosY)
        {
            int newX = juce::roundToInt(newPosX); 
            int newY = juce::roundToInt(newPosY);    
            
            // Limit movement to curve bounds
            newX = juce::jlimit(juce::roundToInt(map_Hz2Pixel(getFreqMin())), 
                                juce::roundToInt(map_Hz2Pixel(getFreqMax())),
                                newX);
            
            newY = juce::jlimit(juce::roundToInt(map_dB2Pixel(0.5f*getRangedB())),
                                juce::roundToInt(map_dB2Pixel(-0.5f*getRangedB())), 
                                newY);
            
            // Limit movement to parameter range
            newX = juce::jlimit(juce::roundToInt(map_Hz2Pixel(band->getMinFreq())), 
                                juce::roundToInt(map_Hz2Pixel(band->getMaxFreq())),
                                newX);
                       
            newY = juce::jlimit(juce::roundToInt(map_dB2Pixel(band->getMaxGain())),
                                juce::roundToInt(map_dB2Pixel(band->getMinGain())), 
                                newY);

            //Get current center before moving
            auto centerAnt =  band->getBounds().getCentre();
            
            // Move the entire BandControl
            band->setCentrePosition(newX, newY);
            
            //Get Hz from X coord
            if(centerAnt.getX() != newX)
            {
                band->setFreq(map_Pixel2Hz(newX));
                if (onFreqChange != nullptr)
                    onFreqChange (band->getID(), band->getFreq());
            }
            
            //Get dB from Y coord
            if(centerAnt.getY() != newY)
            {
                band->setGain(map_Pixel2dB(newY));
                if (onGainChange != nullptr)
                    onGainChange (band->getID(), band->getGain());
            }
        };
        
        band->onGainChanged = [this,band] (uint id, float x,  juce::Point<float> offset)
        {
            // Move the entire BandControl
            band->setCentrePosition(band->getBounds().getCentreX(), (int)map_dB2Pixel(x));
            
            //TELEPORT cursor
            auto parentSpaceCenter = band->getBounds().getCentre().toFloat();
            auto globalCenter = localPointToGlobal (parentSpaceCenter);
            globalCenter.x += offset.x;
            globalCenter.y += offset.y;
            juce::Desktop::getInstance().setMousePosition (globalCenter.roundToInt());
            
            if (onGainChange != nullptr)
                onGainChange (id, x); 
        };
        
        band->onFreqChanged = [this,band] (uint id, float x,  juce::Point<float> offset)
        {
            // Move the entire BandControl
            band->setCentrePosition((int)map_Hz2Pixel(x), band->getBounds().getCentreY());
            
            //TELEPORT cursor
            auto parentSpaceCenter = band->getBounds().getCentre().toFloat();
            auto globalCenter = localPointToGlobal (parentSpaceCenter);
            globalCenter.x += offset.x;
            globalCenter.y += offset.y;
            juce::Desktop::getInstance().setMousePosition (globalCenter.roundToInt());
            
            if (onFreqChange != nullptr)
                onFreqChange (id, x);
        };
        
        band->onQChanged = [this,band] (uint id, float x)
        {
            if (onQChange != nullptr)
                onQChange (id, x);
        };
        
        band->onOnOffChanged = [this,band] (uint id, bool x)
        {
            if (onOnOffChange != nullptr)
                onOnOffChange (id, x);
        };
        
        band->onBandHovered = [this] ()
        {
            hideAllBandsControls();
        };
        
        band->onConfigClicked = [this, band] (uint id)
        {
            //TODO open the extra buttons config region
            DBG("onConfigClicked ID: " << juce::String(id) );
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
    
    //Set all bands positions
    auto bounds = getLocalBounds();
    if (bounds.isEmpty()) return;

    for (const auto& band : EqBands)
    {
        // Pass the cached bounds to your mapping functions
        int x = static_cast<int>(map_Hz2Pixel(band->getFreq()));
        int y = static_cast<int>(map_dB2Pixel(band->getGain()));
        band->setCentrePosition(x, y);
    }
}

void EqCurve::mouseEnter(const juce::MouseEvent& event)
{
    hideAllBandsControls();
}

void EqCurve::hideAllBandsControls()
{
    for (const auto& band : EqBands)
    {
        band->hideChildBalls();
    }
}
