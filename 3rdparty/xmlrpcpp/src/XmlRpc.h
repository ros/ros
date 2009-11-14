// this file modified by Morgan Quigley on 22 April 2008 to add 
// a std::exception-derived class
#ifndef _XMLRPC_H_
#define _XMLRPC_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
// 

#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <string>
#endif

#include "XmlRpcClient.h"
#include "XmlRpcException.h"
#include "XmlRpcServer.h"
#include "XmlRpcServerMethod.h"
#include "XmlRpcValue.h"
#include "XmlRpcUtil.h"

#include <stdexcept>

namespace XmlRpc {


  //! An interface allowing custom handling of error message reporting.
  class XmlRpcErrorHandler {
  public:
    virtual ~XmlRpcErrorHandler() { }

    //! Returns a pointer to the currently installed error handling object.
    static XmlRpcErrorHandler* getErrorHandler() 
    { return _errorHandler; }

    //! Specifies the error handler.
    static void setErrorHandler(XmlRpcErrorHandler* eh)
    { _errorHandler = eh; }

    //! Report an error. Custom error handlers should define this method.
    virtual void error(const char* msg) = 0;

  protected:
    static XmlRpcErrorHandler* _errorHandler;
  };

  //! An interface allowing custom handling of informational message reporting.
  class XmlRpcLogHandler {
  public:
    virtual ~XmlRpcLogHandler() { }

    //! Returns a pointer to the currently installed message reporting object.
    static XmlRpcLogHandler* getLogHandler() 
    { return _logHandler; }

    //! Specifies the message handler.
    static void setLogHandler(XmlRpcLogHandler* lh)
    { _logHandler = lh; }

    //! Returns the level of verbosity of informational messages. 0 is no output, 5 is very verbose.
    static int getVerbosity() 
    { return _verbosity; }

    //! Specify the level of verbosity of informational messages. 0 is no output, 5 is very verbose.
    static void setVerbosity(int v) 
    { _verbosity = v; }

    //! Output a message. Custom error handlers should define this method.
    virtual void log(int level, const char* msg) = 0;

  protected:
    static XmlRpcLogHandler* _logHandler;
    static int _verbosity;
  };

  //! Returns log message verbosity. This is short for XmlRpcLogHandler::getVerbosity()
  int getVerbosity();
  //! Sets log message verbosity. This is short for XmlRpcLogHandler::setVerbosity(level)
  void setVerbosity(int level);


  //! Version identifier
  extern const char XMLRPC_VERSION[];

} // namespace XmlRpc

#endif // _XMLRPC_H_
