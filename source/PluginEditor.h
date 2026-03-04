// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once

#include "PluginProcessor.h"
#include "BinaryData.h"
#include "melatonin_inspector/melatonin_inspector.h"
#include <sap_widgets/sap_widgets.h>

//==============================================================================
class PluginEditor : public juce::AudioProcessorEditor
{
public:
    PluginEditor(PluginProcessor& p);
    ~PluginEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;
    void parentHierarchyChanged() override;

private:
    // This must be static so it can be called in the initializer list
    static std::vector<sap::BandData> createConfig (juce::AudioProcessorValueTreeState& apvts, uint numBands);
    
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    PluginProcessor& processorRef;
    //std::unique_ptr<melatonin::Inspector> inspector;
    //juce::TextButton inspectButton { "Inspect the UI" };
    
    //Add EQ curve
    sap::EqCurve myEqCurve;    
    
    // In your header
    //HoverCircle myCircle;
    
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PluginEditor)
};



