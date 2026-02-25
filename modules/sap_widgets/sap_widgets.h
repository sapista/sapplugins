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

#include <juce_gui_basics/juce_gui_basics.h>

namespace sap
{
    #include "widgets/EqBandControl.h"
    #include "widgets/FreqGrid.h"
    #include "widgets/EqCurve.h"
    
    inline const juce::Colour ColourGrid { 0xff898989 };
    inline const juce::Colour ColourEqBallDefault { 0xaabef8d6 };
    inline const juce::Colour ColourEqBallHover { 0xff32f986 };
    inline const juce::Colour ColourEqBallEdit { 0xfff8d485 };
    inline const juce::Colour ColourTextDark { 0xff000703 };
}
