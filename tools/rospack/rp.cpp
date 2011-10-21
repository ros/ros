/*
 * Copyright (C) 2008, Willow Garage, Inc., Morgan Quigley
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

#include "rp.h"
#include "tinyxml-2.5.3/tinyxml.h"

#include <boost/filesystem.hpp>

#if !defined(WIN32)
  #include <sys/types.h>
  #include <libgen.h>
  #include <limits.h>
  #include <pwd.h>
#endif

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace fs = boost::filesystem;

namespace rospack
{

static const std::string ROSPACK_MANIFEST_NAME = "manifest.xml";
static const std::string ROSSTACK_MANIFEST_NAME = "stack.xml";
static const std::string ROSPACK_CACHE_NAME = "rospack_cache";
static const std::string ROSSTACK_CACHE_NAME = "rosstack_cache";
static const std::string DOTROS_NAME = ".ros";
static const int MAX_CRAWL_DEPTH = 1000;
static const double DEFAULT_MAX_CACHE_AGE = 60.0;

/////////////////////////////////////////////////////////////
// Stackage methods
/////////////////////////////////////////////////////////////
Stackage::Stackage(const std::string& name,
                   const std::string& path,
                   const std::string& manifest_path) :
        name_(name),
        path_(path),
        manifest_path_(manifest_path)
{
}

/////////////////////////////////////////////////////////////
// Rosstackage methods (public/protected)
/////////////////////////////////////////////////////////////
Rosstackage::Rosstackage(std::string manifest_name,
                         std::string cache_name,
                         crawl_direction_t crawl_dir,
                         int max_crawl_depth) :
        manifest_name_(manifest_name),
        cache_name_(cache_name),
        crawl_dir_(crawl_dir),
        max_crawl_depth_(max_crawl_depth),
        crawled_(false)
{
}

void
Rosstackage::debug_dump()
{
  for(std::map<std::string, Stackage*>::const_iterator it = stackages_.begin();
      it != stackages_.end();
      ++it)
  {
    printf("%s:\n  %s\n  %s\n  %s\n",
           it->first.c_str(),
           it->second->name_.c_str(),
           it->second->path_.c_str(),
           it->second->manifest_path_.c_str());
  }
  printf("Total:%d\n", (int)stackages_.size());
}

void
Rosstackage::crawl(const std::vector<std::string>& search_path,
                   bool force)
{
  if(crawled_ && !force)
  {
    printf("alredy crawled\n");
    return;
  }

  if(readCache() && !force)
  {
    printf("loaded cache\n");
    return;
  }

  for(std::vector<std::string>::const_iterator p = search_path.begin();
      p != search_path.end();
      ++p)
  {
    crawlDetail(*p, force, 1);
  }
  
  crawled_ = true;

  writeCache();
  printf("crawled\n");
}

std::string
Rosstackage::find(const std::string& name)
{
  std::map<std::string, Stackage*>::const_iterator it = stackages_.find(name);
  if(it != stackages_.end())
    return it->second->path_;
  else
    return "";
}

/////////////////////////////////////////////////////////////
// Rosstackage methods (private)
/////////////////////////////////////////////////////////////
bool
Rosstackage::isStackage(const std::string& path)
{
  if(!fs::is_directory(path))
    return false;

  for(fs::directory_iterator dit = fs::directory_iterator(path);
      dit != fs::directory_iterator();
      ++dit)
  {
    if(!fs::is_regular_file(dit->path()))
      continue;
    
    if(dit->path().filename() == manifest_name_)
      return true;
  }
  return false;
}

void
Rosstackage::addStackage(const std::string& path)
{
#if !defined(BOOST_FILESYSTEM_VERSION) || (BOOST_FILESYSTEM_VERSION == 2)
  std::string name = fs::path(path).filename();
#else
  // in boostfs3, filename() returns a path, which needs to be stringified
  std::string name = fs::path(path).filename().string();
#endif

  if(stackages_.find(name) != stackages_.end())
  {
    // TODO: optionally notify on duplicates
    return;
  }
  fs::path manifest_path = fs::path(path) / manifest_name_;
  stackages_[name] = new Stackage(name, path, manifest_path.string());
  // TODO
  loadManifest(stackages_[name]);
}

void
Rosstackage::crawlDetail(const std::string& path,
                         bool force,
                         int depth)
{
  if(depth > max_crawl_depth_)
    throw Exception("Maximum depth exceeded during crawl");

  if(!fs::is_directory(path))
    return;

  if(isStackage(path))
  {
    addStackage(path);
    return;
  }

  if(crawl_dir_ == CRAWL_DOWN)
  {
    for(fs::directory_iterator dit = fs::directory_iterator(path);
        dit != fs::directory_iterator();
        ++dit)
    {
      if(fs::is_directory(dit->path()))
        crawlDetail(dit->path().string(), force, depth+1);
    }
  }
  else // dir == CRAWL_UP
  {
    std::string parent = boost::filesystem::path(path).parent_path().string();
    if(parent.size())
      crawlDetail(parent, force, depth+1);
  }
}

void
Rosstackage::loadManifest(Stackage* stackage)
{
  if(stackage->manifest_loaded_)
    return;

  if(!stackage->manifest_.LoadFile(stackage->manifest_path_))
  {
    std::string errmsg = std::string("Error parsing manifest of package ") + 
            stackage->name_ + " at [" + stackage->manifest_path_ + "]";
    throw Exception(errmsg);
  }
  stackage->manifest_loaded_ = true;
}

std::string
Rosstackage::getCachePath()
{
  fs::path cache_path;

  char* ros_home = getenv("ROS_HOME");
  if(ros_home)
    cache_path = ros_home;
  else
  {
    // Get the user's home directory by looking up the password entry based
    // on UID.  If that doesn't work, we fall back on examining $HOME,
    // knowing that that can cause trouble when mixed with sudo (#2884).  
#if defined(WIN32)
    char* home_drive = getenv("HOMEDRIVE");
    char* home_path = getenv("HOMEPATH");
    if(home_drive && home_path) 
      cache_path = fs::path(home_drive) / fs::path(home_path) / fs::path(DOTROS_NAME);
#else // UNIX
    char* home_path;
    struct passwd* passwd_ent;
    // Look up based on effective UID, just in case we got here by set-uid
    if((passwd_ent = getpwuid(geteuid())))
      home_path = passwd_ent->pw_dir;
    else
      home_path = getenv("HOME");
    if(home_path)
      cache_path = fs::path(home_path) / fs::path(DOTROS_NAME);
#endif
  }

  // If it doesn't exist, create the directory that will hold the cache
  if(!fs::is_directory(cache_path))
  {
    try
    {
      fs::create_directory(cache_path);
    }
    catch(fs::filesystem_error& e)
    {
      fprintf(stderr,
              "[rospack] WARNING: cannot create rospack cache directory %s: %s\n",
              cache_path.string().c_str(),
              e.what());
    }
  }
  cache_path /= cache_name_;
  return cache_path.string();
}

bool
Rosstackage::readCache()
{
  if(!validateCache())
    return false;

  std::string cache_path = getCachePath();
  FILE *cache = fopen(cache_path.c_str(), "r");
  if(cache)
  {
    char linebuf[30000];
    for(;;)
    {
      if (!fgets(linebuf, sizeof(linebuf), cache))
        break; // error in read operation
      if (linebuf[0] == '#')
        continue;
      char* newline_pos = strchr(linebuf, '\n');
      if(newline_pos)
        *newline_pos = 0;
      addStackage(linebuf);
    }
    fclose(cache);
    return true;
  }
  else
    return false;
}

// TODO: replace the contents of the method with some fancy cross-platform
// boost thing.
void
Rosstackage::writeCache()
{
  // Write the results of this crawl to the cache file.  At each step, give
  // up on error, printing a warning to stderr.
  std::string cache_path = getCachePath();
  if(!cache_path.size())
  {
    fprintf(stderr, "[rospack] No location available to write cache file.  Try setting ROS_HOME or HOME.\n");
  }
  else
  {
    char tmp_cache_dir[PATH_MAX];
    char tmp_cache_path[PATH_MAX];
    strncpy(tmp_cache_dir, cache_path.c_str(), sizeof(tmp_cache_dir));
#if defined(_MSC_VER)
    // No dirname on Windows; use _splitpath_s instead
    char drive[_MAX_DRIVE], dir[_MAX_DIR], fname[_MAX_FNAME], ext[_MAX_EXT];
    _splitpath_s(tmp_cache_dir, drive, _MAX_DRIVE, dir, _MAX_DIR, fname, _MAX_FNAME,
                 ext, _MAX_EXT);
    char full_dir[_MAX_DRIVE + _MAX_DIR];
    _makepath_s(full_dir, _MAX_DRIVE + _MAX_DIR, drive, dir, NULL, NULL);
    snprintf(tmp_cache_path, sizeof(tmp_cache_path), "%s\\.rospack_cache.XXXXXX", full_dir);
#elif defined(__MINGW32__)
    char* temp_name = tempnam(dirname(tmp_cache_dir),".rospack_cache.");
    snprintf(tmp_cache_path, sizeof(tmp_cache_path), temp_name);
    delete temp_name;
#else
    snprintf(tmp_cache_path, sizeof(tmp_cache_path), "%s/.rospack_cache.XXXXXX", dirname(tmp_cache_dir));
#endif
#if defined(__MINGW32__)
    // There is no equivalent of mkstemp or _mktemp_s on mingw, so we resort to a slightly problematic
    // tempnam (above) and mktemp method. This has the miniscule chance of a race condition.
    int fd = open(tmp_cache_path, O_RDWR | O_EXCL | _O_CREAT, 0644);
    if (fd < 0)
    {
      fprintf(stderr, "[rospack] Unable to create temporary cache file %s: %u\n",
              tmp_cache_path, errno);
    }
    else
    {
      FILE *cache = fdopen(fd, "w");
#elif defined(WIN32)
    if (_mktemp_s(tmp_cache_path, PATH_MAX) != 0)
    {
      fprintf(stderr,
              "[rospack] Unable to generate temporary cache file name: %u",
              GetLastError());
    }
    else
    {
      FILE *cache = fopen(tmp_cache_path, "w");
#else
    int fd = mkstemp(tmp_cache_path);
    if (fd < 0)
    {
      fprintf(stderr, "[rospack] Unable to create temporary cache file %s: %s\n", 
              tmp_cache_path, strerror(errno));
    }
    else
    {
      FILE *cache = fdopen(fd, "w");
#endif
      if (!cache)
      {
        fprintf(stderr, "[rospack] Unable open cache file %s: %s\n", 
                tmp_cache_path, strerror(errno));
      }
      else
      {
        // TODO: remove writing of ROS_ROOT
        char *rr = getenv("ROS_ROOT");
        fprintf(cache, "#ROS_ROOT=%s\n", rr);

        char *rpp = getenv("ROS_PACKAGE_PATH");
        fprintf(cache, "#ROS_PACKAGE_PATH=%s\n", (rpp ? rpp : ""));
        for(std::map<std::string, Stackage*>::const_iterator it = stackages_.begin();
            it != stackages_.end();
            ++it)
          fprintf(cache, "%s\n", it->second->path_.c_str());
        fclose(cache);
        if(fs::exists(cache_path))
          remove(cache_path.c_str());
        if(rename(tmp_cache_path, cache_path.c_str()) < 0)
        {
          fprintf(stderr, "[rospack] Error: failed to rename cache file %s to %s: %s\n", 
                  tmp_cache_path, cache_path.c_str(), strerror(errno));
        }
      }
    }
  }
}

bool
Rosstackage::validateCache()
{
  std::string cache_path = getCachePath();
  // first see if it's new enough
  double cache_max_age = DEFAULT_MAX_CACHE_AGE;
  const char *user_cache_time_str = getenv("ROS_CACHE_TIMEOUT");
  if(user_cache_time_str)
    cache_max_age = atof(user_cache_time_str);
  if(cache_max_age == 0.0)
    return false;
  struct stat s;
  if(stat(cache_path.c_str(), &s) == 0)
  {
    double dt = difftime(time(NULL), s.st_mtime);
    // Negative cache_max_age means it's always new enough.  It's dangerous
    // for the user to set this, but rosbash uses it.
    if ((cache_max_age > 0.0) && (dt > cache_max_age))
      return false;
  }
  // try to open it 
  FILE* cache = fopen(cache_path.c_str(), "r");
  if(!cache)
    return false; // it's not readable by us. sad.

  // see if ROS_PACKAGE_PATH matches
  char linebuf[30000];
  bool ros_package_path_ok = false;
  const char *ros_package_path = getenv("ROS_PACKAGE_PATH");
  for(;;)
  {
    if(!fgets(linebuf, sizeof(linebuf), cache))
      break;
    linebuf[strlen(linebuf)-1] = 0; // get rid of trailing newline
    if (linebuf[0] == '#')
    {
      if(!strncmp("#ROS_PACKAGE_PATH=", linebuf, 18))
      {
        if(!ros_package_path)
        {
          if(!strlen(linebuf+18))
            ros_package_path_ok = true;
        }
        else if(!strcmp(linebuf+18, getenv("ROS_PACKAGE_PATH")))
          ros_package_path_ok = true;
      }
    }
    else
      break; // we're out of the header. nothing more matters to this check.
  }
  fclose(cache);
  return ros_package_path_ok;
}

/////////////////////////////////////////////////////////////
// Rospack methods
/////////////////////////////////////////////////////////////
Rospack::Rospack() :
        Rosstackage(ROSPACK_MANIFEST_NAME,
                    ROSPACK_CACHE_NAME,
                    CRAWL_DOWN,
                    MAX_CRAWL_DEPTH)
{
}

void Rospack::crawl(const std::vector<std::string>& search_path,
                    bool force)
{
  Rosstackage::crawl(search_path, force);
}

/////////////////////////////////////////////////////////////
// Rosstack methods
/////////////////////////////////////////////////////////////
Rosstack::Rosstack() :
        Rosstackage(ROSSTACK_MANIFEST_NAME,
                    ROSSTACK_CACHE_NAME,
                    CRAWL_UP,
                    MAX_CRAWL_DEPTH)
{
}

void Rosstack::crawl(const std::vector<std::string>& search_path,
                     bool force)
{
  Rosstackage::crawl(search_path, force);
}

}
