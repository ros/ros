/////////////////////////////////////////////////////////////////////////////
// Name:        controls.i
// Purpose:     Control (widget) classes for wxPython
//
// Author:      Robin Dunn
//
// Created:     7-July-1997
// RCS-ID:      $Id: gdi.i,v 1.72 2006/10/11 04:07:14 RD Exp $
// Copyright:   (c) 2003 by Total Control Software
// Licence:     wxWindows license
/////////////////////////////////////////////////////////////////////////////

%module(package="wx") _gdi

//---------------------------------------------------------------------------

%{
#include "wx/wxPython/wxPython.h"
#include "wx/wxPython/pyclasses.h"
    
%}

//---------------------------------------------------------------------------

%import core.i
%pythoncode { wx = _core }

MAKE_CONST_WXSTRING_NOSWIG(EmptyString);


// Include all the files that make up this module

%include _gdiobj.i
%include _colour.i
%include _palette.i
%include _pen.i 
%include _brush.i
%include _bitmap.i
%include _icon.i
%include _cursor.i
%include _region.i
%include _font.i
%include _intl.i
%include _dc.i
%include _graphics.i
%include _overlay.i
%include _imaglist.i
%include _stockobjs.i
%include _effects.i
%include _renderer.i
%include _pseudodc.i

//---------------------------------------------------------------------------
%init %{
    // Work around a chicken/egg problem in drawlist.cpp
    wxPyDrawList_SetAPIPtr();
%}

//---------------------------------------------------------------------------
