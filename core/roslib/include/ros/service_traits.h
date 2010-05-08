/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_SERVICE_TRAITS_H
#define ROSCPP_SERVICE_TRAITS_H

namespace ros
{
namespace service_traits
{

/**
 * \brief Specialize to provide the md5sum for a service
 */
template<typename M>
struct MD5Sum
{
  static const char* value()
  {
    return M::__s_getServerMD5Sum().c_str();
  }

  static const char* value(const M& m)
  {
    return m.__getServerMD5Sum().c_str();
  }
};

/**
 * \brief Specialize to provide the datatype for a service
 */
template<typename M>
struct DataType
{
  static const char* value()
  {
    return M::__s_getServiceDataType().c_str();
  }

  static const char* value(const M& m)
  {
    return m.__getServiceDataType().c_str();
  }
};

/**
 * \brief return MD5Sum<M>::value();
 */
template<typename M>
inline const char* md5sum()
{
  return MD5Sum<M>::value();
}

/**
 * \brief return DataType<M>::value();
 */
template<typename M>
inline const char* datatype()
{
  return DataType<M>::value();
}

/**
 * \brief return MD5Sum<M>::value(m);
 */
template<typename M>
inline const char* md5sum(const M& m)
{
  return MD5Sum<M>::value(m);
}

/**
 * \brief return DataType<M>::value();
 */
template<typename M>
inline const char* datatype(const M& m)
{
  return DataType<M>::value(m);
}

} // namespace message_traits
} // namespace ros

#endif // ROSCPP_SERVICE_TRAITS_H
