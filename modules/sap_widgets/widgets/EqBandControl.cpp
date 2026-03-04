// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Pere Ràfols Soler

EqBall::EqBall(const juce::String& units,  bool horizontalDrag, ParameterData param_data)
    : str_units(units),
    bHorizontalDrag(horizontalDrag),
    paramData(param_data),
    bIsEditing(false)
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

    // Check if it's specifically the Left Button AND the mouse is over/dragging us
    bool isLeftDragging = mods.isLeftButtonDown() && isMouseOverOrDragging();
    
    // Change color based on hover state
    if(bIsEditing)
    {
        g.setColour(sap::ColourEqBallEdit);
    }
    else
    {
        g.setColour((isMouseOver() || isLeftDragging) ? sap::ColourEqBallHover : sap::ColourEqBallDefault);
    }
    
    auto area = getLocalBounds().toFloat();    
    float cornerSize = getHeight() * 0.5f; 
    g.fillRoundedRectangle (area, cornerSize);
    g.setColour (juce::Colours::white.withAlpha(0.7f));
    g.drawRoundedRectangle (area.reduced(0.5f), cornerSize, 1.0f);
    
    //Text label
    g.setColour (sap::ColourTextDark);
    auto myFont = juce::FontOptions ("Verdana", 15.0f, juce::Font::bold);
    g.setFont (myFont); // Set font height in pixels
    
    if(bIsEditing)
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
    if(bIsEditing) return; //Abort if in edit mode
    
    grabKeyboardFocus();
    bIsEditing = true;
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
    if (!bIsEditing) return false;

    if (key == juce::KeyPress::returnKey)
    {
        // Finish editing
        bIsEditing = false;
        
                
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
    if (bIsEditing)
    {
        bIsEditing = false;
        repaint();
    }
}

void EqBall::mouseWheelMove (const juce::MouseEvent& e, const juce::MouseWheelDetails& w)
{
    if(bIsEditing) return; //Abort if in edit mode
    
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
    
    setMouseCursor (juce::MouseCursor::NoCursor);
}

void EqBall::mouseEnter(const juce::MouseEvent& event) 
{
    if(bIsEditing) return; //Abort if in edit mode
    repaint();
}

void EqBall::mouseExit(const juce::MouseEvent& event)
{
    if(bIsEditing) return; //Abort if in edit mode
    repaint();
}

void EqBall::mouseDrag (const juce::MouseEvent& event)
{   
    if(bIsEditing) return; //Abort if in edit mode
    
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
    if(bIsEditing) return; //Abort if in edit mode
    
    if (event.mods.isLeftButtonDown())
    {
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
        
        setMouseCursor (juce::MouseCursor::NoCursor);
    }
}

void EqBall::mouseUp (const juce::MouseEvent& event)
{
    if(bIsEditing) return; //Abort if in edit mode
    
    setMouseCursor (juce::MouseCursor::NormalCursor);
    
    // Map the center of the circle to the Screen
    auto localCenter = getLocalBounds().getCentre();

    // Convert coords in the component to monitor coords
    auto globalCenter = localPointToGlobal (localCenter);

    juce::Desktop::getInstance().setMousePosition (globalCenter); 
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

//=================================================== EQ BAND
EqBandControl::EqBandControl(BandData band)
    : id(band.ID), 
    BallGain("dB", false, band.gain),
    BallFreq("Hz", true, band.freq),
    BallQ("Q", true, band.q),
    bBallIsDragging(false)
{
    addChildComponent (BallGain);
    addChildComponent (BallFreq);
    addChildComponent (BallQ);
    
    //Hard-coded layout
    //setBounds(0, 0, 152, 56);
    setSize (152, 56);
    BallGain.setTopLeftPosition(7, 1);
    BallFreq.setTopLeftPosition(85, 7);
    BallQ.setTopLeftPosition(85, 30);
    
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
    };
    
    BallQ.onValueChange = [this] (float newValue){
        if(onQChanged != nullptr)
        {
            onQChanged(id, newValue);
        }
    };

    
}

void EqBandControl::paint (juce::Graphics& g) 
{
    // Get the current global mouse modifiers
    auto mods = juce::ModifierKeys::getCurrentModifiers();

    // Check if it's specifically the Left Button AND the mouse is over/dragging us
    bool isLeftDragging = mods.isLeftButtonDown() && isMouseOverOrDragging();
    
    //TODO add tree state colors: defauks, hover, dragging
    
    // Change color based on hover state
    g.setColour((isMouseOver() || isLeftDragging) ? sap::ColourEqBallHover : sap::ColourEqBallDefault);
    auto area = getLocalBounds().toFloat();  
    auto circleArea = juce::Rectangle<float> (ball_diameter, ball_diameter).withCentre (area.getCentre());
    g.fillEllipse (circleArea);
    
    //TODO Add the white border!
    
    //TEST CROSS //TODO comment out!
    auto center = getLocalBounds().getCentre().toFloat();
    const float armLength = 5.0f;
    g.setColour (juce::Colours::red);
    g.drawLine (center.x, center.y - armLength, 
                center.x, center.y + armLength, 
                1.0f);
    g.drawLine (center.x - armLength, center.y, 
                center.x + armLength, center.y, 
                1.0f);
    
}

void EqBandControl::mouseEnter(const juce::MouseEvent& event)
{   
    //This event will trigger to force all bands controls event this one! So must be called first!
    if(onBandHovered != nullptr)
        onBandHovered();
 
    BallGain.setVisible(true);
    BallFreq.setVisible(true);
    BallQ.setVisible(true);
    toFront (true); 
    repaint();
}

void EqBandControl::mouseExit(const juce::MouseEvent& event)
{
    
    repaint();
}

void EqBandControl::mouseDown (const juce::MouseEvent& event)
{
    if (event.mods.isLeftButtonDown())
    {
        //Get coords only if mouse started the drag on the ball
        auto center = getLocalBounds().getCentre().toFloat();
        auto startPos = event.getMouseDownPosition().toFloat();
        float distanceFromCenter = center.getDistanceFrom (startPos);
        
        if (distanceFromCenter <= ball_diameter*0.6f)
        {
            bBallIsDragging = true;
        }
    }
}

void EqBandControl::mouseUp (const juce::MouseEvent& event)
{
    //TODO use it?
}

void EqBandControl::mouseDrag (const juce::MouseEvent& event)
{
    if (! event.mods.isAnyMouseButtonDown())
    {
        bBallIsDragging = false;
        return;
    }
    
    if(bBallIsDragging)
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
    bBallIsDragging = false;
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
    
    //TODO possible improvement: add a timmer, when a band is unselected... wait a few ms to make it disapear?
    BallFreq.setVisible(false);
    BallQ.setVisible(false);
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
