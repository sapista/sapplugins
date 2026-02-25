// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once

class EqBall : public juce::Component
{
public:
    EqBall(const juce::String& units, bool horizontalDrag);
    void paint (juce::Graphics& g) override;
    void mouseDoubleClick(const juce::MouseEvent& e) override;
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    void mouseEnter(const juce::MouseEvent& event) override;
    void mouseExit(const juce::MouseEvent& event)override;
    void mouseDrag (const juce::MouseEvent& event) override;
    void mouseDown (const juce::MouseEvent& event) override;
    void mouseUp (const juce::MouseEvent& event) override;
    bool keyPressed (const juce::KeyPress& key) override;
    void focusLost (FocusChangeType cause) override;
    
    // Data accessors
    void setValue(float x);

    // Signals
    std::function<void (float)> onValueChange;
    std::function<void (float, float)> onMouseDrag;
    
    
private:
    juce::String str_units;
    bool bHorizontalDrag;
    float value = 0.0f;
    juce::Point<float> offset;
    
    bool bIsEditing;
    juce::String str_currentInputString;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBall)
};


class EqBandControl : public juce::Component
{
public:
    EqBandControl(const uint ID);
    
    //TODO revise uneeded methods and remove
    void paint (juce::Graphics& g) override;
    //void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    //bool hitTest(int x, int y) override;
    void mouseEnter(const juce::MouseEvent& event) override;
    void mouseExit(const juce::MouseEvent& event)override;

    // Use a std::function to handle the event elsewhere (like in your MainComponent)
    //std::function<void(bool)> onHover;
    
    // Data accessors
    void setGain(float x);
    void setFreq(float x);
    
    
    // Signals
    std::function<void (float, float)> onMouseDrag;
    std::function<void (float, juce::Point<float>)> onGainChanged;
    std::function<void (float, juce::Point<float>)> onFreqChanged;
    std::function<void (float)> onQChanged;
    
    
private:
    const uint id;
    static constexpr float ball_diameter = 22.0f;
    EqBall BallGain, BallFreq, BallQ;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBandControl)
};
