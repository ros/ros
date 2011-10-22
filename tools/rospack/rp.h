/*
 * Copyright (C) 2008, Willow Garage, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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


#ifndef ROSPACK_ROSPACK_H
#define ROSPACK_ROSPACK_H

#include "tinyxml-2.5.3/tinyxml.h"

#include <string>
#include <map>
#include <vector>
#include <stdexcept>

namespace rospack
{

class Stackage
{
  public:
    // \brief name of the stackage
    std::string name_;
    // \brief absolute path to the stackage
    std::string path_;
    // \brief absolute path to the stackage manifest
    std::string manifest_path_;
    // \brief have we already loaded the manifest?
    bool manifest_loaded_;
    // \brief TinyXML structure, filled in during parsing
    rospack_tinyxml::TiXmlDocument manifest_;

    Stackage(const std::string& name,
             const std::string& path,
             const std::string& manifest_path);
};

class Package : public Stackage
{
};

class Stack : public Stackage
{
};

typedef enum
{
  CRAWL_UP,
  CRAWL_DOWN
} crawl_direction_t;

class Exception : public std::runtime_error
{
  public:
    Exception(const std::string& what)
            : std::runtime_error(what)
    {}
};

class Rosstackage
{
  private:
    std::string manifest_name_;
    std::string cache_name_;
    crawl_direction_t crawl_dir_;
    int max_crawl_depth_;

    bool crawled_;
    bool isStackage(const std::string& path);
    void addStackage(const std::string& path);
    void crawlDetail(const std::string& path,
                     bool force,
                     int depth);
    void loadManifest(Stackage* stackage);
    std::string getCachePath();
    bool readCache();
    void writeCache();
    bool validateCache();

  protected:
    std::map<std::string, Stackage*> stackages_;
    void crawl(const std::vector<std::string>& search_path, bool force);

  public:
    Rosstackage(std::string manifest_name,
                std::string cache_name,
                crawl_direction_t crawl_dir,
                int max_crawl_depth);

    std::string find(const std::string& name); 

    void debug_dump();
};

class Rospack : public Rosstackage
{
  public:
    // TODO: make private
    void crawl(const std::vector<std::string>& search_path,
               bool force);
    Rospack();
};

class Rosstack : public Rosstackage
{
  public:
    // TODO: make private
    void crawl(const std::vector<std::string>& search_path,
               bool force);
    Rosstack();
};


// Simple console output helpers
void rospack_warn(const std::string& name, 
                  const std::string& msg,
                  bool append_errno = false);
void rospack_error(const std::string& name, 
                   const std::string& msg,
                   bool append_errno = false);

} // namespace rospack


#endif
