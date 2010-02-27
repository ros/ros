//  Copyright (c) 2009 Helge Bahmann
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#include <boost/config.hpp>

#if defined(__GNUC__) && (defined(__i386__) || defined(__amd64__))

	#include "detail/gcc-x86.hpp"

#elif defined(__GNUC__) && defined(__alpha__)

	#include "detail/gcc-alpha.hpp"

#elif defined(__GNUC__) && (defined(__POWERPC__) || defined(__PPC__))

	#include "detail/gcc-ppc.hpp"

#elif defined(BOOST_USE_WINDOWS_H) || defined(_WIN32_CE) || defined(BOOST_MSVC) || defined(BOOST_INTEL_WIN) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)

	#include "detail/interlocked.hpp"

#else

	#include "detail/generic-cas.hpp"

#endif
