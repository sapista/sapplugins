// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once

struct ParameterData
{
    juce::String paramID;
    float default_value;
    juce::NormalisableRange<float> range;
};

struct BandData
{
    uint ID;
    ParameterData freq;
    ParameterData gain;
    ParameterData q;
};

class EqBall : public juce::Component
{
public:
    EqBall(const juce::String& units, bool horizontalDrag, ParameterData param_data);
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
    float getValue();
    float getMinValue();
    float getMaxValue();

    // Signals
    std::function<void (float)> onValueChange;
    std::function<void (float, float)> onMouseDrag;
    
    
private:
    juce::String str_units;
    bool bHorizontalDrag;
    ParameterData paramData;
    float value = 0.0f;
    juce::Point<float> offset;
    bool bIsEditing;
    juce::String str_currentInputString;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBall)
};


class EqBandControl : public juce::Component
{
public:
    EqBandControl(BandData band);
    
    void paint (juce::Graphics& g) override;
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override; 
    void mouseEnter(const juce::MouseEvent& event) override;
    void mouseExit(const juce::MouseEvent& event)override;
    void mouseDown (const juce::MouseEvent& event) override;
    void mouseUp (const juce::MouseEvent& event) override;
    void mouseDrag (const juce::MouseEvent& event) override;
    void focusLost (FocusChangeType cause) override;
     
    // Data accessors
    uint getID();
    void setGain(float x);
    void setFreq(float x);
    void setQ(float x);
    float getGain();
    float getFreq();
    float getQ();
    float getMinFreq();
    float getMaxFreq();
    float getMinGain();
    float getMaxGain();
    void hideChildBalls();
        
    // Signals
    std::function<void ()> onBandHovered;
    std::function<void (float, float)> onMouseDrag;
    std::function<void (uint, float, juce::Point<float>)> onGainChanged;
    std::function<void (uint, float, juce::Point<float>)> onFreqChanged;
    std::function<void (uint, float)> onQChanged;    
    
private:
    const uint id;
    static constexpr float ball_diameter = 22.0f;
    EqBall BallGain, BallFreq, BallQ;
    bool bBallIsDragging;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBandControl)
};
