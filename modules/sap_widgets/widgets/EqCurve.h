// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once
#include <vector>
#include <memory>

class EqCurve : public FreqGrid
{
public:
    EqCurve(const uint band_count);
    void paint (juce::Graphics& g) override;
    void resized() override;
    
    //Data accessors
    //TODO
    
    // Signals
    //TODO
    
private:
    const uint bandCount;
    std::vector<std::unique_ptr<sap::EqBandControl>> EqBands;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqCurve)
};

