/*******************************************************************************
 BEGIN_JUCE_MODULE_DECLARATION
  ID:               sap_widgets
  vendor:           SapPlugins
  version:          0.1.0
  name:             SAP Widgets
  description:      Custom UI widgets for the SapPlugins project.
  website:          https://github.com/sapista/sapplugins
  license:          GPL3

  dependencies:     juce_gui_basics, juce_graphics
 END_JUCE_MODULE_DECLARATION
*******************************************************************************/

#pragma once

// 1. First, include the JUCE requirements for this module
#include <juce_gui_basics/juce_gui_basics.h>
#include <juce_graphics/juce_graphics.h>
#include <juce_events/juce_events.h>

// 2. Then include melatonin (since it's a dependency)
#include <melatonin_blur/melatonin_blur.h>

namespace sap
{
    enum class VisualState
    {
        Default,
        Hovered,
        Dragging,
        TextEditting
    };
    
    namespace ShadowOpacity
    {
        inline constexpr float Default = 0.0f;
        inline constexpr float Hovered = 0.4f;
        inline constexpr float Dragging = 0.5f;
    }
    
    #include "widgets/EqBandControl.h"
    #include "widgets/FreqGrid.h"
    #include "widgets/EqCurve.h"
    
    
    inline const juce::Colour ColourGrid { 0xff898989 };
    inline const juce::Colour ColourEqBallDefault { 0xaabef8d6 };
    inline const juce::Colour ColourEqBallHover { 0xff32f986 };
    inline const juce::Colour ColourEqBallEdit { 0xfff8d485 };
    inline const juce::Colour ColourTextDark { 0xff442102 };
    
}
