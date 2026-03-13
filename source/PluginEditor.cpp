// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#include "PluginEditor.h"

PluginEditor::PluginEditor (PluginProcessor& p)
    : AudioProcessorEditor (&p), 
    processorRef (p),
    myEqCurve (createConfig (p.apvts, PluginProcessor::num_Bands))
{
    juce::ignoreUnused (processorRef);

    /*addAndMakeVisible (inspectButton);
    
        
    // this chunk of code instantiates and opens the melatonin inspector
    inspectButton.onClick = [&] {
        if (!inspector)
        {
            inspector = std::make_unique<melatonin::Inspector> (*this);
            inspector->onClose = [this]() { inspector.reset(); };
        }

        inspector->setVisible (true);
    };
    */
    
    addAndMakeVisible (myEqCurve);
    
    
    // Signal Slots
    myEqCurve.onGainChange = [this] (uint id, float newValue)
    {
        //TODO connect to audio processor
        DBG("onGainChange ID: " << juce::String(id) << " value: " << newValue);
    };
    
    myEqCurve.onFreqChange = [this] (uint id, float newValue)
    {
        //TODO connect to audio processor
        DBG("onFreqChange ID: " << juce::String(id) << " value: " << newValue);
    };
    
    myEqCurve.onQChange = [this] (uint id, float newValue)
    {
        //TODO connect to audio processor
        DBG("onQChange ID: " << juce::String(id) << " value: " << newValue);
    };
    
    myEqCurve.onOnOffChange = [this] (uint id, bool newValue)
    {
        //TODO connect to audio processor
        DBG("onOnOffChange ID: " << juce::String(id) << " value: " << (newValue ? "On" : "Off"));
    };
    
    //Allow the user to drag the corner
    setResizable (true, true); 
    setResizeLimits (400, 300, 1680, 1200);
    setSize (1000, 600);
    
}

std::vector<sap::BandData> PluginEditor::createConfig(juce::AudioProcessorValueTreeState& apvts, uint numBands)
{
    std::vector<sap::BandData> config;
    config.reserve(static_cast<std::size_t> (numBands)); // Optimization: prevent reallocations

    for (uint i = 0; i < numBands; ++i)
    {
        auto suffix = juce::String(i + 1); 
        
        //Syntax order: ID, param_key_str, default value or last laoded in daw, range
        config.push_back({ 
            i + 1,
            { "band_freq" + suffix, apvts.getRawParameterValue("band_freq" + suffix)->load(), apvts.getParameterRange("band_freq" + suffix) }, 
            { "band_gain" + suffix, apvts.getRawParameterValue("band_gain" + suffix)->load(), apvts.getParameterRange("band_gain" + suffix) },
            { "band_q"    + suffix, apvts.getRawParameterValue("band_q" + suffix)->load(), apvts.getParameterRange("band_q"    + suffix) }
            });
    }
    return config;
    }

PluginEditor::~PluginEditor()
{
}

//Need to let the Standalone gui work in linux
void PluginEditor::parentHierarchyChanged()
{
    // Check if we are actually attached to a native window
    if (auto* peer = getPeer()) 
    {
        // Re-apply the size to the native window peer
        // This stops the 400x300 -> 0x0 collapse
        peer->getBounds(); 
        setSize (1000, 600);
        
        //DBG("Parent hierarchy changed: Peer is now valid.");
    }
}


void PluginEditor::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

    auto area = getLocalBounds();
    g.setColour (juce::Colours::white);
    g.setFont (16.0f);
    auto helloWorld = juce::String ("Hello from ") + PRODUCT_NAME_WITHOUT_VERSION + " v" VERSION + " running in " + CMAKE_BUILD_TYPE;
    g.drawText (helloWorld, area.removeFromTop (150), juce::Justification::centred, false);
}

void PluginEditor::resized()
{
    //DBG ("Resized called: " << getWidth() << " x " << getHeight());
    // layout the positions of your child components here
    auto area = getLocalBounds();
    myEqCurve.setBounds(area); //TODO investigate how to limit to a portion of the area
    //TODO this was to make room for melatonin inspector
    //area.removeFromBottom(50);
    //inspectButton.setBounds (getLocalBounds().withSizeKeepingCentre(100, 50));
}
