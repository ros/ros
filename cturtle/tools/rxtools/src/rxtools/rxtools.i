%module rxtools
%include "std_string.i"

%{
#include "ros/types.h"
#include "wx/wxPython/wxPython.h"
#include "wx/wxPython/pyclasses.h"
#include "rosout_filter.h"
#include "rosout_panel.h"
#include "init_roscpp.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%pythonAppend RosoutPanel "self._setOORInfo(self)"

%include rosout_filter.h
%include rosout_generated.h
%include rosout_panel.h
%include init_roscpp.h

%init %{

%}
