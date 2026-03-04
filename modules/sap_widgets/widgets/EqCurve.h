// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once
#include <vector>
#include <memory>

class EqCurve : public FreqGrid
{
public:
    EqCurve(const std::vector<BandData>& bands);
    void paint (juce::Graphics& g) override;
    void resized() override;
    void mouseEnter(const juce::MouseEvent& event) override;
    
    //Data accessors
    //TODO
    
    // Signals
    std::function<void (uint, float)> onGainChange;
    std::function<void (uint, float)> onFreqChange;
    std::function<void (uint, float)> onQChange;
    //TODO add bypass, mid-side, LR... etc...
    
private:
    void hideAllBandsControls();
    std::vector<std::unique_ptr<sap::EqBandControl>> EqBands;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqCurve)
};

