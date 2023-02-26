/*
 * DISTRHO Plugin Framework (DPF)
 * Copyright (C) 2012-2019 Filipe Coelho <falktx@falktx.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose with
 * or without fee is hereby granted, provided that the above copyright notice and this
 * permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD
 * TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER
 * IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "DistrhoPluginInfo.h"
#include "DistrhoUI.hpp"
#include "Window.hpp"

START_NAMESPACE_DISTRHO

// -----------------------------------------------------------------------------------------------------------

class GainUI : public UI
{
  public:
    GainUI()
        : UI(405, 256),
          fScale(1.0f)
    {
        fSampleRate = getSampleRate();
        setGeometryConstraints(405, 256, true);
    }

protected:
   /* --------------------------------------------------------------------------------------------------------
    * DSP/Plugin Callbacks */

   /**
      A parameter has changed on the plugin side.
      This is called by the host to inform the UI about parameter changes.
    */
    void parameterChanged(uint32_t, float value) override
    {
        fParamGain = value;
        repaint();
    }

   /* --------------------------------------------------------------------------------------------------------
    * DSP/Plugin Callbacks (optional) */

   /**
      Optional callback to inform the UI about a sample rate change on the plugin side.
    */
    void sampleRateChanged(double newSampleRate) override
    {
        fSampleRate = newSampleRate;
        repaint();
    }

   /* --------------------------------------------------------------------------------------------------------
    * Widget Callbacks */

   /**
      The NanoVG drawing function.
    */
    void onNanoDisplay() override
    {
        
        
        beginPath();
        fillColor(0, 255, 255);
        roundedRect(0, 0, 100, 100, 2);
        fill();
        
        
        beginPath();
        fillColor(255, 0, 0);
        textAlign(ALIGN_RIGHT|ALIGN_TOP);
        textBox(100, 100, 100 * fScale, "testing Text!");
        closePath();
        
        
        const float lineHeight = 20 * fScale;

        fontSize(15.0f * fScale);
        textLineHeight(lineHeight);

        float x = 0.0f * fScale;
        float y = 15.0f * fScale;

        // sample rate
        drawLeft(x, y, "Sample Rate:");
        drawRight(x, y, getTextBufFloat(fSampleRate));
        y+=lineHeight;

        // nothing
        y+=lineHeight;

    }


    void onResize(const ResizeEvent& ev) override
    {
        fScale = static_cast<float>(ev.size.getHeight())/256.0f;
        UI::onResize(ev);
    }

    // -------------------------------------------------------------------------------------------------------

private:
    // Parameters
    float  fParamGain;
    double fSampleRate;

    // UI stuff
    FontId fFont;
    float fScale;

    // temp buf for text
    char fStrBuf[0xff+1];

    // helpers for putting text into fStrBuf and returning it
    const char* getTextBufInt(const int value)
    {
        std::snprintf(fStrBuf, 0xff, "%i", value);
        return fStrBuf;
    }

    const char* getTextBufFloat(const float value)
    {
        std::snprintf(fStrBuf, 0xff, "%.1f", value);
        return fStrBuf;
    }


    // helpers for drawing text
    void drawLeft(const float x, const float y, const char* const text)
    {
        beginPath();
        fillColor(200, 200, 200);
        textAlign(ALIGN_RIGHT|ALIGN_TOP);
        textBox(x, y, 100 * fScale, text);
        closePath();
    }

    void drawRight(const float x, const float y, const char* const text)
    {
        beginPath();
        fillColor(255, 255, 255);
        textAlign(ALIGN_LEFT|ALIGN_TOP);
        textBox(x + (105 * fScale), y, 100 * fScale, text);
        closePath();
    }

   /**
      Set our UI class as non-copyable and add a leak detector just in case.
    */
    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(GainUI)
};

/* ------------------------------------------------------------------------------------------------------------
 * UI entry point, called by DPF to create a new UI instance. */

UI* createUI()
{
    return new GainUI();
}

// -----------------------------------------------------------------------------------------------------------

END_NAMESPACE_DISTRHO
