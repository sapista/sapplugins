// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#include "PluginEditor.h"

PluginEditor::PluginEditor (PluginProcessor& p)
    : AudioProcessorEditor (&p), processorRef (p), myEqCurve(5)
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
    
    
    //TODO delete me
    // In your constructor
    //addAndMakeVisible(myCircle);
    //myCircle.setBounds(100, 100, 50, 50);
    
    /*
    myCircle.onHover = [this](bool isHovered) {
    if (isHovered) {
        DBG("Mouse entered the circle!");
    } else {
        DBG("Mouse left the circle!");
    }
    };
    */
    

    //Allow the user to drag the corner
    setResizable (true, true); 
    setResizeLimits (400, 300, 1680, 1200);
    setSize (1000, 600);
    
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
