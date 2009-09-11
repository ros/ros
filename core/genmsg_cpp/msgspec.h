/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef MSG_GEN_H
#define MSG_GEN_H

#include <string>
#include <vector>

class msg_var;
class var_constant;

class msg_spec
{
public:
  std::string spec_file, class_name, spec_name, md5sum, package, pkg_path;
  std::string full_definition;
  std::string server_md5sum;
  std::string service_datatype;
  std::vector<msg_var *> vars;
  std::vector<var_constant *> constant_vars;
  msg_spec(const std::string &spec_file, const std::string &_package,
           const std::string &spec_name, const std::string &_pkg_path,
           bool is_filename = true, bool _is_root = false);
  bool process_line(char *line, int linenum);
  static bool is_primitive(const std::string &type);
  static bool is_array(const std::string &type);
  static bool is_integer(const std::string &type);
  void emit_cpp_class(FILE *f, bool for_srv = false, const std::string &service_name = std::string());
  void emit_cpp_includes(FILE *f);
  std::string serializationLength_func();
  std::string to_list_func();
  bool is_fixed_length();
  std::vector<msg_spec *> cpp_types(std::vector<msg_spec *> existing_types);
  bool is_root;
  std::string equals(const std::string &prefix, int indent = 0);
  std::string test_populate(const std::string &prefix, int indent = 0);
  msg_var *make_var(const std::string &type, const std::string &name);
};

#endif

