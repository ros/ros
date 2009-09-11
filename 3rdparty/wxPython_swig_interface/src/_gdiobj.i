/////////////////////////////////////////////////////////////////////////////
// Name:        _gdiobj.i
// Purpose:     SWIG interface for wxGDIObject
//
// Author:      Robin Dunn
//
// Created:     13-Sept-2003
// RCS-ID:      $Id: _gdiobj.i,v 1.4 2006/04/18 21:56:24 RD Exp $
// Copyright:   (c) 2003 by Total Control Software
// Licence:     wxWindows license
/////////////////////////////////////////////////////////////////////////////

// Not a %module

//---------------------------------------------------------------------------
%newgroup

MustHaveApp(wxGDIObject);

class wxGDIObject : public wxObject {
public:
    wxGDIObject();
    ~wxGDIObject();

    bool IsNull();

};

//---------------------------------------------------------------------------
