// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

#pragma once

#ifdef IDE_PARSER
  #include "../sap_widgets.h"
  using namespace sap;
#endif

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
    std::function<void ()> onHovered;
    
private:
    juce::String str_units;
    bool bHorizontalDrag;
    ParameterData paramData;
    float value = 0.0f;
    juce::Point<float> offset;
    VisualState currentState = VisualState::Default;
    juce::String str_currentInputString;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBall)
};

class OnOffBall : public juce::Button
{
public:
    OnOffBall();
    void paintButton (juce::Graphics& g, bool isHovered, bool isDown) override;
    
    // Data accessors
    void setValue(bool x);
    bool getValue();

    // Signals
    std::function<void (bool)> onValueChange;
    std::function<void ()> onHovered;
    
private:    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (OnOffBall)
};

class ConfigBall : public juce::Button
{
public:
    ConfigBall();
    void paintButton (juce::Graphics& g, bool isHovered, bool isDown) override;
    
    // Signals
    std::function<void ()> onHovered;
    
private:
    juce::Image wrenchCache;
    void renderWrenchToCache();
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ConfigBall)
};

class EqBandControl final: public juce::Component, public juce::Timer
{
public:
    EqBandControl(BandData band);
    
    void paint (juce::Graphics& g) override;
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override; 
    void mouseEnter(const juce::MouseEvent& event) override;
    bool hitTest(int x, int y) override;
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
    std::function<void (uint, bool)> onOnOffChanged;    
    std::function<void (uint)> onConfigClicked;
    
private:
    const uint id;
    static constexpr float ball_diameter = 22.0f;
    EqBall BallGain, BallFreq, BallQ;
    OnOffBall BallOnOff;
    ConfigBall BallConfig;
    VisualState currentState = VisualState::Default;
    bool mouseInBall(const juce::MouseEvent& event);
    
    //Drop shadow
    juce::Path shadowPath;
    melatonin::DropShadow shadow = { juce::Colours::white, ball_diameter, { 0,0 }, 1.0f }; //{ color, radius, offset, spread }
    float shadowOpacity = 0.0f;
    float targetOpacity = 0.0f;
    void setShadowOpacity(float target);
    void timerCallback() override;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqBandControl)
};
