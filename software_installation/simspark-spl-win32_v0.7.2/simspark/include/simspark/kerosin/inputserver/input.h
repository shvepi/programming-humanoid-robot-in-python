/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

this file is part of rcssserver3D
Fri May 9 2003
Copyright (C) 2002,2003 Koblenz University
Copyright (C) 2003 RoboCup Soccer Server 3D Maintenance Group
$Id$

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; version 2 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#ifndef KEROSIN_INPUTCONST_H
#define KEROSIN_INPUTCONST_H

#include <kerosin/kerosin_defines.h>

namespace kerosin
{

/**  this defines the input data structure, encapsulating all
     input events generated by the devices.
*/
class KEROSIN_API Input
{
public:
    typedef int TInputCode;

    enum EModifiers
        {
            eNone   = 0x0000,
            eLShift = 0x0001,
            eRShift = 0x0002,
            eShift  = 0x0003,       // eLShift|eRShift
            eLCtrl  = 0x0040,
            eRCtrl  = 0x0080,
            eCtrl   = 0x00c0,       // eLCtrl|eRCtrl
            eLAlt   = 0x0100,
            eRAlt   = 0x0200,
            eAlt    = 0x0300,       // eLAlt|eRAlt
            eNum    = 0x1000,
            eCaps   = 0x2000,
        };

    //! this enumerates different tpyes of input events.
    enum EType
        {
            eUnknown,
            eButton,        // all buttons of a keyboard and mouse buttons
            eAxis,          // two mouse axis, the mouse wheel and the
            // time axis
            eUser           // a user specified input event
        };

public:
    //! this initializes values indicating an invalid input event
    Input(EType t = eUnknown, TInputCode c=0, int i=-1);

    //! marks this input structure to represent a key press event
    void SetKeyPress();

    //! marks this input structure to represent a key release event
    void SetKeyRelease();

    //! returns true if the input represents a key press event
    bool GetKeyPress() const;

        //! returns true if the input represents a key release event
    bool GetKeyRelease() const;

public:
    //! this indicates the input data type
    EType mType;

    //! the IC_ code of the button or the axis (see inputconst.h)
    TInputCode mCode;

    /** this is a user defined value the Input event evaluated to,
        or -1 to indicate a raw Input event.
    */
    int mId;

    //! union for additional data
    union
    {
        /** this is used to encode a button event. currently only
            1 for 'pressed' and 0 for 'released' are used
        */
        long l;

        //! this is used to encode a position on an axis.
        float f;
    } mData;

    unsigned int mModState;
public:

    // numbers
    static const TInputCode IC_1;
    static const TInputCode IC_2;
    static const TInputCode IC_3;
    static const TInputCode IC_4;
    static const TInputCode IC_5;
    static const TInputCode IC_6;
    static const TInputCode IC_7;
    static const TInputCode IC_8;
    static const TInputCode IC_9;
    static const TInputCode IC_0;

    // function keys
    static const TInputCode IC_F1;
    static const TInputCode IC_F2;
    static const TInputCode IC_F3;
    static const TInputCode IC_F4;
    static const TInputCode IC_F5;
    static const TInputCode IC_F6;
    static const TInputCode IC_F7;
    static const TInputCode IC_F8;
    static const TInputCode IC_F9;
    static const TInputCode IC_F10;
    static const TInputCode IC_F11;
    static const TInputCode IC_F12;

    // alphabet
    static const TInputCode IC_A;
    static const TInputCode IC_B;
    static const TInputCode IC_C;
    static const TInputCode IC_D;
    static const TInputCode IC_E;
    static const TInputCode IC_F;
    static const TInputCode IC_G;
    static const TInputCode IC_H;
    static const TInputCode IC_I;
    static const TInputCode IC_J;
    static const TInputCode IC_K;
    static const TInputCode IC_L;
    static const TInputCode IC_M;
    static const TInputCode IC_N;
    static const TInputCode IC_O;
    static const TInputCode IC_P;
    static const TInputCode IC_Q;
    static const TInputCode IC_R;
    static const TInputCode IC_S;
    static const TInputCode IC_T;
    static const TInputCode IC_U;
    static const TInputCode IC_V;
    static const TInputCode IC_W;
    static const TInputCode IC_X;
    static const TInputCode IC_Y;
    static const TInputCode IC_Z;

    // keypad
    static const TInputCode IC_KP0;
    static const TInputCode IC_KP1;
    static const TInputCode IC_KP2;
    static const TInputCode IC_KP3;
    static const TInputCode IC_KP4;
    static const TInputCode IC_KP5;
    static const TInputCode IC_KP6;
    static const TInputCode IC_KP7;
    static const TInputCode IC_KP8;
    static const TInputCode IC_KP9;
    static const TInputCode IC_KP_DECIMAL;
    static const TInputCode IC_KP_DIVIDE;
    static const TInputCode IC_KP_MULTIPLY;
    static const TInputCode IC_KP_MINUS;
    static const TInputCode IC_KP_PLUS;
    static const TInputCode IC_KP_ENTER;

    // arrows + home/end pad
    static const TInputCode IC_UP;
    static const TInputCode IC_DOWN;
    static const TInputCode IC_LEFT;
    static const TInputCode IC_RIGHT;
    static const TInputCode IC_INSERT;
    static const TInputCode IC_DELETE;
    static const TInputCode IC_HOME;
    static const TInputCode IC_END;
    static const TInputCode IC_PAGEUP;
    static const TInputCode IC_PAGEDOWN;

    // key state modifier keys
    static const TInputCode IC_NUMLOCK;
    static const TInputCode IC_CAPSLOCK;
    static const TInputCode IC_SCROLLOCK;
    static const TInputCode IC_LSHIFT;
    static const TInputCode IC_RSHIFT;
    static const TInputCode IC_LCTRL;
    static const TInputCode IC_RCTRL;
    static const TInputCode IC_LALT;
    static const TInputCode IC_RALT;
    static const TInputCode IC_LSUPER;      // Left "Windows" key
    static const TInputCode IC_RSUPER;      // Right "Windows" key

    // other keys (cursor control, punctuation)
    static const TInputCode IC_ESCAPE;
    static const TInputCode IC_PRINT;
    static const TInputCode IC_PAUSE;
    static const TInputCode IC_GRAVE;
    static const TInputCode IC_MINUS;
    static const TInputCode IC_EQUALS;
    static const TInputCode IC_BACKSLASH;
    static const TInputCode IC_BACKSPACE;

    static const TInputCode IC_TAB;
    static const TInputCode IC_LBRACKET;
    static const TInputCode IC_RBRACKET;
    static const TInputCode IC_RETURN;

    static const TInputCode IC_SEMICOLON;
    static const TInputCode IC_APOSTROPHE;

    static const TInputCode IC_OEM_102;     // German <>|
    static const TInputCode IC_COMMA;
    static const TInputCode IC_PERIOD;
    static const TInputCode IC_SLASH;

    static const TInputCode IC_SPACE;

    // mouse buttons
    static const TInputCode IC_MOUSE_LEFT;   // left
    static const TInputCode IC_MOUSE_RIGHT;  // right
    static const TInputCode IC_MOUSE_MIDDLE; // middle

    //mouse axis
    static const TInputCode IC_AXISX;
    static const TInputCode IC_AXISY;
    static const TInputCode IC_AXISZ;

    // timer
    static const TInputCode IC_AXIST;
};

} // kerosin

#endif // KEROSIN_INPUTCONST_H
