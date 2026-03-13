// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

//TODO delete this!
#ifdef IDE_PARSER
  #include "EqBandControl.h"
#endif

using namespace sap;

EqBall::EqBall(const juce::String& units,  bool horizontalDrag, ParameterData param_data)
    : str_units(units),
    bHorizontalDrag(horizontalDrag),
    paramData(param_data)
{
    const int w = str_units.length() * 12 + 41;
    setBounds(0, 0, w, 20); 
    setWantsKeyboardFocus(true);
    value = paramData.default_value;
}

void EqBall::paint (juce::Graphics& g) 
{
    // Get the current global mouse modifiers
    auto mods = juce::ModifierKeys::getCurrentModifiers();    
    
    switch (currentState)
    {
        case VisualState::Default:
            g.setColour (sap::ColourEqBallDefault);
            break;
            
        case VisualState::Hovered:
            g.setColour (sap::ColourEqBallHover);
            break;
            
        case VisualState::Dragging:
            g.setColour (sap::ColourEqBallEdit);
            break;
        
        case VisualState::TextEditting:
            g.setColour(sap::ColourEqBallEdit);
            break;
    }
    
    auto area = getLocalBounds().toFloat();    
    float cornerSize = getHeight() * 0.5f; 
    g.fillRoundedRectangle (area, cornerSize);
    g.setColour (sap::ColourTextDark);
    g.drawRoundedRectangle (area.reduced(0.5f), cornerSize, 1.0f);
    
    //Text label
    g.setColour (sap::ColourTextDark);
    auto myFont = juce::FontOptions ("Verdana", 15.0f, juce::Font::bold);
    g.setFont (myFont); // Set font height in pixels
    
    if(currentState == VisualState::TextEditting)
    {
        g.drawText (str_currentInputString + "|", getLocalBounds(), juce::Justification::centred);
    }
    else
    {
        if(str_units == "Hz")
        {
            g.drawText ((value >= 1000 ? juce::String (value/1000, 1) + " k" : juce::String (value, 1) + " ") + str_units, getLocalBounds(), juce::Justification::centred);
        }
        else
        {
            g.drawText (juce::String(value, 1) + " " + str_units, getLocalBounds(), juce::Justification::centred);
        }
    }
    
}

void EqBall::mouseDoubleClick(const juce::MouseEvent& e)
{
    if(currentState == VisualState::TextEditting) return; //Abort if in edit mode
    
    grabKeyboardFocus();
    currentState = VisualState::TextEditting;
    if(str_units == "Hz")
    {
        str_currentInputString = (value >= 1000 ? juce::String (value/1000, 1) + "k" : juce::String (value, 1));
    }
    else
    {
        str_currentInputString = juce::String(value, 1); 
    }
    repaint();
}

bool EqBall::keyPressed (const juce::KeyPress& key)
{
    if (currentState != VisualState::TextEditting)
        return false;

    if (key == juce::KeyPress::returnKey)
    {
        // Finish editing
        if(isMouseOver())
        {
            currentState = VisualState::Hovered;
        }
        else
        {
            currentState = VisualState::Default;
        }
                
        float newValue;
        if (str_currentInputString.endsWith("k"))
        {
            newValue = str_currentInputString.dropLastCharacters(1).getFloatValue() * 1000.0f;
        }
        else
        {
            newValue = str_currentInputString.getFloatValue();
        }
        
        setValue(newValue); //Set the param value and limit to a valid range
        if (onValueChange != nullptr)
            onValueChange (value);
        
        return true; 
    }
    
    if (key == juce::KeyPress::backspaceKey)
    {
        str_currentInputString = str_currentInputString.dropLastCharacters(1);
        repaint();
        return true;
    }

    // Capture numbers and dots
    juce::juce_wchar character = key.getTextCharacter();
    juce::String validChars = "0123456789.kK-";
    if (validChars.containsChar(character))
    {
        // Don't allow multiple 'k's (e.g., "2kk" is invalid)
        if ((character == 'k' || character == 'K') && (str_currentInputString.containsAnyOf("kK") || str_units != "Hz"))
            return true;
        
        // k must be the last char when used
        if(str_currentInputString.containsAnyOf("kK"))
            return true;
        
        // The negative sign is only for the gain bandControl
        if( character == '-' && str_units != "dB")
            return true;
        
        // Don't allow multiple '-'
        if (character == '-' && str_currentInputString.containsAnyOf("-") )
            return true;
        
        // Minuts sign only allowed at the begin of str
        if (character == '-' && str_currentInputString.isNotEmpty())
            return true;
        
        str_currentInputString += character;
        repaint();
        return true; // We handled the key
    }

    return false; // Let other components handle keys we don't want
}

void EqBall::focusLost (FocusChangeType cause)
{
    currentState = VisualState::Default;
    repaint();
}

void EqBall::mouseWheelMove (const juce::MouseEvent& e, const juce::MouseWheelDetails& w)
{
    if(currentState == VisualState::TextEditting) 
        return; //Abort if in edit mode
    
    if(str_units == "Hz")
    {
        float increment = w.deltaY > 0 ? 0.02f : -0.02f;
        float x = std::log(value);
        x += increment;
        setValue(std::exp(x));
    }
    else
    {
        float increment = w.deltaY > 0 ? 0.1f : -0.1f;
        setValue(value + increment);  
    }
     
    if (onValueChange != nullptr)
        onValueChange (value);
    
    //setMouseCursor (juce::MouseCursor::NoCursor);
    if (bHorizontalDrag)
    {
        setMouseCursor (juce::MouseCursor::LeftRightResizeCursor);
    }
    else
    {
        setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
    }
}

void EqBall::mouseEnter(const juce::MouseEvent& event) 
{
    if(currentState == VisualState::TextEditting)
        return; //Abort if in edit mode
    
    currentState = VisualState::Hovered;
    repaint();
    
    if (onHovered != nullptr)
                    onHovered();
}

void EqBall::mouseExit(const juce::MouseEvent& event)
{
    if(currentState == VisualState::TextEditting)
        return; //Abort if in edit mode
        
    currentState = VisualState::Default;
    repaint();

}

void EqBall::mouseDrag (const juce::MouseEvent& event)
{   
    if(currentState == VisualState::TextEditting)
        return; //Abort if in edit mode
    
    if (event.mods.isLeftButtonDown())
    {   
        if (auto* bandControl = getParentComponent())
        {
            // Get the mouse position relative to the GRANDPARENT
            if (auto* grandParent = bandControl->getParentComponent())
            {
                auto mouseInGrandParent = event.getEventRelativeTo (grandParent).position;
                auto parentPos = bandControl->getPosition();
                
                parentPos.x += 0.5f*bandControl->getWidth();
                parentPos.y += 0.5f*bandControl->getHeight();

                // New absolute position in the Grandparent's space
                float absoluteNewPosX, absoluteNewPosY;
                if (bHorizontalDrag)
                {
                    absoluteNewPosX = mouseInGrandParent.x - offset.x;
                    absoluteNewPosY = parentPos.y;
                    
                    if (str_units == "Q")
                    {
                        static float Q_x_ant = 0.0f;
                        float xdelta = absoluteNewPosX  - Q_x_ant;
                        Q_x_ant = absoluteNewPosX;                       
                        setValue(value + 0.01f*xdelta);
                        if (onValueChange != nullptr)
                            onValueChange (value);
                    }
                }   
                else
                {
                    absoluteNewPosX = parentPos.x;
                    absoluteNewPosY = mouseInGrandParent.y - offset.y;
                }
                
                if (onMouseDrag != nullptr)
                    onMouseDrag (absoluteNewPosX, absoluteNewPosY);
                
            }
        }
    }
}


void EqBall::mouseDown (const juce::MouseEvent& event)
{
    if(currentState == VisualState::TextEditting)
        return; //Abort if in edit mode
    
    if (event.mods.isLeftButtonDown())
    {
        currentState = VisualState::Dragging;
         if (auto* bandControl = getParentComponent())
        {
            // Get the mouse position relative to the GRANDPARENT
            if (auto* grandParent = bandControl->getParentComponent())
            {
                auto mouseInGrandParent = event.getEventRelativeTo (grandParent).position;
                //auto parentPos = bandControl->getPosition();
                auto parentBounds = bandControl->getBounds();
                offset.x = mouseInGrandParent.x - parentBounds.getCentreX();
                offset.y = mouseInGrandParent.y - parentBounds.getCentreY();
            }
        }
        
        //setMouseCursor (juce::MouseCursor::NoCursor);
        if (bHorizontalDrag)
        {
            setMouseCursor (juce::MouseCursor::LeftRightResizeCursor);
        }
        else
        {
            setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
        }
        
        repaint();
    }
}

void EqBall::mouseUp (const juce::MouseEvent& event)
{
    if(currentState == VisualState::TextEditting)
        return; //Abort if in edit mode
    
    if(isMouseOver())
    {
        currentState = VisualState::Hovered;
    }
    else
    {
        currentState = VisualState::Default;
    }
    
    setMouseCursor (juce::MouseCursor::NormalCursor);
    
    // Map the center of the circle to the Screen
    auto localCenter = getLocalBounds().getCentre();

    // Convert coords in the component to monitor coords
    auto globalCenter = localPointToGlobal (localCenter);

    juce::Desktop::getInstance().setMousePosition (globalCenter); 
    
    repaint();
}


// Set new value
void EqBall::setValue(float x)
{    
    value = juce::jlimit(paramData.range.start, paramData.range.end, x);   
    repaint();
}

// Get value
float EqBall::getValue()
{
    return value;
}

float EqBall::getMinValue()
{
    return paramData.range.start;
}

float EqBall::getMaxValue()
{
    return paramData.range.end;
}

//=================================================== On Off Button 
OnOffBall::OnOffBall()
 : juce::Button ("")
{
    setClickingTogglesState (true);
    setToggleState (false, juce::dontSendNotification);
    setBounds(0, 0, 27, 20); 

    onStateChange = [this]
    {
        if ((onHovered != nullptr) && isMouseOver())
                onHovered();
        repaint();
    };
    
    onClick = [this]
    {
        bool isActive = getToggleState();
        if (onValueChange != nullptr)
            onValueChange (isActive);
    };
}
    
void OnOffBall::paintButton (juce::Graphics& g, bool isHovered, bool isDown)
{
    juce::Colour c;
    juce::String t = "Off";
    if (getToggleState()) // If button is "ON"
    {
        t = "On";
        if (isHovered)
        {
            c = sap::ColourEqBallEdit.brighter(2.0f);
        }
        else
        {
            c = sap::ColourEqBallEdit;
        }
    }
    else
    {
       if (isHovered)
        {
            c = sap::ColourEqBallHover;;
        }
        else
        {
            c = sap::ColourEqBallDefault;
        } 
    }
    
    g.setColour (c);
    auto area = getLocalBounds().toFloat();    
    float cornerSize = getHeight() * 0.3f; 
    g.fillRoundedRectangle (area, cornerSize);
    g.setColour (sap::ColourTextDark);
    g.drawRoundedRectangle (area.reduced(0.5f), cornerSize, 1.0f);
    
    //Text label
    g.setColour (sap::ColourTextDark);
    auto myFont = juce::FontOptions ("Verdana", 14.0f, juce::Font::bold);
    g.setFont (myFont); // Set font height in pixels
    g.drawText (t, getLocalBounds(), juce::Justification::centred);
}

void OnOffBall::setValue(bool x)
{
    setToggleState (x, juce::dontSendNotification);
    //TODO is this calling the repaint method? maybe i need to enable notification....
}

bool OnOffBall::getValue()
{
    return getToggleState();
}

//=================================================== Config Button 
ConfigBall::ConfigBall()
 : juce::Button ("")
{
    wrenchCache = juce::Image (juce::Image::ARGB, 20, 20, true);
    renderWrenchToCache();
    setBounds(0, 0, 20, 20); 

    onStateChange = [this]
    {
        if ((onHovered != nullptr) && isMouseOver())
                onHovered();

        repaint();
    };
}
    
void ConfigBall::paintButton (juce::Graphics& g, bool isHovered, bool isDown)
{
    juce::Colour c;
    if (isDown) 
    {
        if (isHovered)
        {
            c = sap::ColourEqBallEdit.brighter(2.0f);
        }
        else
        {
            c = sap::ColourEqBallEdit;
        }
    }
    else
    {
       if (isHovered)
        {
            c = sap::ColourEqBallHover;;
        }
        else
        {
            c = sap::ColourEqBallDefault;
        } 
    }
    
    g.setColour (c);
    auto area = getLocalBounds().toFloat();    
    float cornerSize = getHeight() * 0.3f; 
    g.fillRoundedRectangle (area, cornerSize);
    g.setColour (sap::ColourTextDark);
    g.drawRoundedRectangle (area.reduced(0.5f), cornerSize, 1.0f);
    
    //Add the wrench
    g.drawImageAt (wrenchCache, 0, 0);
}

void ConfigBall::renderWrenchToCache()
{
    juce::Path p;

    // Dimensions calibrated for a modern, refined look within a 20x20 area.
    const float headWidth = 9.0f;
    const float headHeight = 7.0f;
    const float jawOpeningWidth = 4.0f;
    const float handleWidth = 3.0f;
    const float totalLength = 16.0f; // Total length including the head

    // Define the silhouette centered on (0,0) before rotation.
    // The asymmetric shape creates the distinct head and single jaw.

    // 1. Draw the asymmetric "Head" profile
    p.startNewSubPath (-headWidth / 2.0f, -headHeight / 2.0f); // Top Left
    p.lineTo (headWidth / 2.0f, -headHeight / 2.0f);           // Top Right
    p.lineTo (headWidth / 2.0f, headHeight / 2.0f);            // Bottom Right (at the jaw base)
    p.lineTo (-handleWidth / 2.0f, headHeight / 2.0f);          // Start of handle indent
    p.lineTo (-handleWidth / 2.0f, totalLength / 2.0f);         // Bottom of handle
    p.lineTo (-headWidth / 2.0f, totalLength / 2.0f);          // Bottom Left of handle
    p.closeSubPath(); // Connect back to Top Left

    // 2. Define and apply the "Jaw Cutout" (the 'C' shape)
    juce::Path jawCutout;
    jawCutout.addRectangle (0.0f, // Centered cutout start
                             -headHeight / 2.0f - 1.0f, // Slightly extended above top edge
                             jawOpeningWidth, 
                             headHeight / 1.5f); // Cutout depth
    
    // Rotate the JAW itself 45 degrees relative to the head 
    // for the specific crescent look visible in the render
    jawCutout.applyTransform (juce::AffineTransform::rotation (juce::MathConstants<float>::pi / 4.0f));

    p.setUsingNonZeroWinding (false); // Required for subtraction logic
    // Subtract the jaw (requires NonZeroWinding to be false)
    p.addPath (jawCutout); 

    // 3. Apply corner rounding for the finished icon look
    p = p.createPathWithRoundedCorners (0.8f); // Refined softness seen in image

    // --- The 45 Degree Rotation ---
    // Finally, rotate the entire finished adjustable wrench
    p.applyTransform (juce::AffineTransform::rotation (juce::MathConstants<float>::pi / 4.0f));
    
    //Render to image
    juce::Graphics g (wrenchCache);

    // 4. Transform and Center
    // Since the image is 20x20, the center is (10, 10)
    auto transform = juce::AffineTransform::translation (10.0f, 10.0f);

    // 5. Render the path into the image
    g.setColour (juce::Colours::white);
    g.fillPath (p, transform);
}

//=================================================== EQ BAND
EqBandControl::EqBandControl(BandData band)
    : id(band.ID), 
    BallGain("dB", false, band.gain),
    BallFreq("Hz", true, band.freq),
    BallQ("Q", true, band.q)
{    
    addChildComponent (BallGain);
    addChildComponent (BallFreq);
    addChildComponent (BallQ);
    addChildComponent (BallOnOff);
    addChildComponent (BallConfig);
    
    //Hard-coded layout
    setSize (152, 56);
    BallGain.setTopLeftPosition(7, 1);
    BallFreq.setTopLeftPosition(85, 7);
    BallQ.setTopLeftPosition(85, 30);
    BallOnOff.setTopLeftPosition(38, 20);
    BallConfig.setTopLeftPosition(55, 38);
    
    //Shadow
    auto fullArea = getLocalBounds().toFloat();
    auto size = std::min (fullArea.getWidth(), fullArea.getHeight());
    size -= 14.0f;
    auto squareArea = fullArea.withSizeKeepingCentre (size, size);
    shadowPath.addEllipse (squareArea);
    
    // Signal Slots
    BallGain.onValueChange = [this] (float newValue)
    {
        if(onGainChanged != nullptr)
        {
            juce::Point<float> offset = BallGain.getBounds().getCentre().toFloat() - getLocalBounds().getCentre().toFloat();
            onGainChanged(id, newValue, offset);
        }
    };
    
    BallGain.onMouseDrag = [this] (float newPosX, float newPosY)
    {            
        if(onMouseDrag != nullptr)
        {
            onMouseDrag(newPosX, newPosY);
        }
        setShadowOpacity(ShadowOpacity::Dragging);
    };
    
    BallGain.onHovered = [this]()
    {
        setShadowOpacity(ShadowOpacity::Hovered);
    };
        
    BallFreq.onValueChange = [this] (float newValue){ 
        if(onFreqChanged != nullptr)
        {
            juce::Point<float> offset = BallFreq.getBounds().getCentre().toFloat() - getLocalBounds().getCentre().toFloat();
            onFreqChanged(id, newValue, offset);
        }
    };
    
    BallFreq.onMouseDrag = [this] (float newPosX, float newPosY)
    {            
        if(onMouseDrag != nullptr)
        {
            onMouseDrag(newPosX, newPosY);
        }
        setShadowOpacity(ShadowOpacity::Dragging);
    };
    
    BallFreq.onHovered = [this]()
    {
        setShadowOpacity(ShadowOpacity::Hovered);
    };
    
    BallQ.onValueChange = [this] (float newValue)
    {
        if(onQChanged != nullptr)
        {
            onQChanged(id, newValue);
        }
    };
    
    BallQ.onHovered = [this]()
    {
        setShadowOpacity(ShadowOpacity::Hovered);
    };
    
    BallOnOff.onValueChange = [this] (bool newValue)
    {
        if(onOnOffChanged != nullptr)
        {
            onOnOffChanged(id, newValue);
        }
    };
     
    BallOnOff.onHovered = [this]()
    {
        setShadowOpacity(ShadowOpacity::Hovered);
    };
    
    BallConfig.onClick = [this]()
    {
        if(onConfigClicked != nullptr)
        {
            onConfigClicked(id);
        }
    };
    
    BallConfig.onHovered = [this]()
    {
        setShadowOpacity(ShadowOpacity::Hovered);
    };
}

void EqBandControl::paint (juce::Graphics& g) 
{
    //Drop-shadow    
    shadow.setOpacity(shadowOpacity);
    shadow.render(g, shadowPath);
    
    auto area = getLocalBounds().toFloat();  
    auto circleArea = juce::Rectangle<float> (ball_diameter, ball_diameter).withCentre (area.getCentre());
    
    switch (currentState)
    {
        case VisualState::Default:
            g.setColour (sap::ColourEqBallDefault);
            break;
            
        case VisualState::Hovered:
            g.setColour (sap::ColourEqBallHover);
            break;
            
        case VisualState::Dragging:
            g.setColour (sap::ColourEqBallEdit);
            circleArea = circleArea.reduced(2.0f); // Make the ball "shrink" slightly when pressed
            break;
        
        case VisualState::TextEditting:
            DBG("Error in EqBandControl::paint(): No valid VisualState::TextEditting." );
            break;
    }
    
    g.fillEllipse (circleArea);
    g.setColour (sap::ColourTextDark);
    g.drawEllipse (circleArea.reduced (0.5f), 1.0f);
    
    //Text label ID at the center of the ball
    g.setColour (sap::ColourTextDark);
    auto myFont = juce::FontOptions ("Verdana", 17.0f, juce::Font::bold);
    g.setFont (myFont); // Set font height in pixels
    g.drawText (juce::String(id), circleArea, juce::Justification::centred);
    

}

bool EqBandControl::hitTest (int x, int y)
{
    //Check if the point is within any of our child components
    for (auto* child : getChildren())
    {
        if (child->isVisible() && child->getBounds().contains(x, y))
            return true;
    }

    //check pointer in the ball
    auto center = getLocalBounds().getCentre().toFloat();
    return center.getDistanceFrom ({ (float)x, (float)y }) <= ball_diameter*1.2f;
}

void EqBandControl::mouseEnter(const juce::MouseEvent& event)
{   
    currentState = VisualState::Hovered;
    
    //This event will trigger to force all bands controls event this one! So must be called first!
    if(onBandHovered != nullptr)
        onBandHovered();
 
    BallGain.setVisible(true);
    BallFreq.setVisible(true);
    BallQ.setVisible(true);
    BallOnOff.setVisible(true);
    BallConfig.setVisible(true);
    toFront (true);
    setShadowOpacity(ShadowOpacity::Hovered);
    repaint();
}

void EqBandControl::mouseExit(const juce::MouseEvent& event)
{
    currentState = VisualState::Default;
    setShadowOpacity(ShadowOpacity::Default);
    repaint();
}

void EqBandControl::mouseDown (const juce::MouseEvent& event)
{
    if (event.mods.isLeftButtonDown() && mouseInBall(event))
    {
        currentState = VisualState::Dragging;
        setShadowOpacity(ShadowOpacity::Dragging);
        setMouseCursor (juce::MouseCursor::UpDownLeftRightResizeCursor);
        repaint();
    }
}

void EqBandControl::mouseUp (const juce::MouseEvent& event)
{
    currentState = VisualState::Hovered; 
    setShadowOpacity(ShadowOpacity::Hovered);
    setMouseCursor (juce::MouseCursor::NormalCursor);
    repaint();
}

void EqBandControl::mouseDrag (const juce::MouseEvent& event)
{
    if (! event.mods.isAnyMouseButtonDown())
    {
        if (mouseInBall(event))
        {
            currentState = VisualState::Hovered;
            setShadowOpacity(ShadowOpacity::Hovered);
        }
        else
        {
            currentState = VisualState::Default;
            setShadowOpacity(ShadowOpacity::Default);
        }
        repaint();
        return;
    }
    
    if(currentState == VisualState::Dragging)
    {
        auto parentEvent = event.getEventRelativeTo (getParentComponent());
        auto mouseInParent = parentEvent.position;
        if(onMouseDrag != nullptr)
        {
            onMouseDrag(mouseInParent.x, mouseInParent.y);
        }
    }
}

void EqBandControl::mouseWheelMove (const juce::MouseEvent& event, const juce::MouseWheelDetails& det)
{
    //Start wheel action only if mouse on the ball
    auto center = getLocalBounds().getCentre().toFloat();
    auto startPos = event.getMouseDownPosition().toFloat();
    float distanceFromCenter = center.getDistanceFrom (startPos);
    
    if (distanceFromCenter <= ball_diameter*0.6f)
    {
        BallQ.mouseWheelMove(event, det);
    }
}

void EqBandControl::focusLost (FocusChangeType cause)
{
    // If we lose focus, act as if the user let go of the mouse
    currentState = VisualState::Default;
    setShadowOpacity(ShadowOpacity::Default);
    repaint();
}

uint EqBandControl::getID()
{
    return id;
}

void EqBandControl::setGain(float x)
{
    BallGain.setValue(x);
}

float EqBandControl::getGain()
{
    return BallGain.getValue();
}

void EqBandControl::setFreq(float x)
{
    BallFreq.setValue(x);    
}

float EqBandControl::getFreq()
{
    return BallFreq.getValue();
}

void EqBandControl::setQ(float x)
{
    BallQ.setValue(x);    
}

float EqBandControl::getQ()
{
    return BallQ.getValue();
}

void EqBandControl::hideChildBalls()
{
    BallGain.setVisible(false);
    BallFreq.setVisible(false);
    BallQ.setVisible(false);
    BallOnOff.setVisible(false);
    BallConfig.setVisible(false);
    setShadowOpacity(ShadowOpacity::Default);
}

float EqBandControl::getMinFreq()
{
    return BallFreq.getMinValue();
}

float EqBandControl::getMaxFreq()
{
    return BallFreq.getMaxValue();
}

float EqBandControl::getMinGain()
{
    return BallGain.getMinValue();
}

float EqBandControl::getMaxGain()
{
    return BallGain.getMaxValue();
}

bool EqBandControl::mouseInBall(const juce::MouseEvent& event)
{
    //Start wheel action only if mouse on the ball
    auto center = getLocalBounds().getCentre().toFloat();
    auto startPos = event.getMouseDownPosition().toFloat();
    float distanceFromCenter = center.getDistanceFrom (startPos);
    
    return (distanceFromCenter <= ball_diameter*0.6f);
}

void EqBandControl::timerCallback()
{
    // Smoothly approach the target
    auto gain = 0.15f; // Adjust for fade speed
    shadowOpacity += (targetOpacity - shadowOpacity) * gain;

    // Stop timer when close enough to save CPU
    if (std::abs (shadowOpacity - targetOpacity) < 0.001f)
    {
        shadowOpacity = targetOpacity;
        stopTimer();
    }
    
    repaint();
}

void EqBandControl::setShadowOpacity(float target)
{
    targetOpacity = target;
    if (! isTimerRunning()) startTimerHz (60);
}
