///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ implementation for ROS.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <set>
#include <cstdio>
#include <cerrno>
#include <cassert>
#include <stdexcept>
#include <libgen.h> // for dirname(3)
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/param.h>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include "msgspec.h"
#include "utils.h"

using namespace std;

// TODO: migrate this into msg_spec
static bool is_integer(const string &type);
// TODO: migrate this into msg_spec
static bool is_float(const string &type);
static bool is_bool(const string& type);

void write_depfile( const char *filename, const set<string> &deps );
set<string> &read_depfile( const char *filename, set<string> &deps );

list<string> g_accessors;

class msg_var
{
public:
  string name;
  msg_spec *parent;
  msg_var(const string &_name, msg_spec *_parent)
  : name(_name), parent(_parent) { }
  virtual ~msg_var() { }
  virtual string cpp_decl() = 0;
  virtual string length_expr() = 0;
  virtual bool is_fixed_length() = 0;
  virtual vector<string> cpp_ctor_clauses() = 0;
  virtual vector<string> cpp_copy_ctor_initializers() = 0;
  virtual string cpp_copy_ctor_code() = 0;
  virtual string cpp_assign_code() = 0;
  virtual string cpp_dtor_code() = 0;
  virtual string helper_funcs() = 0;
  virtual string serialization_code() = 0;
  virtual string deserialization_code() = 0;
  virtual string cpp_type_name() = 0;
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *>) = 0;
  virtual string equals(const string &prefix, int indent = 0) = 0;
  virtual string test_populate(const string &prefix, int indent = 0) = 0;
};


class var_complex : public msg_var
{
public:
  string type_spec_str, pkg_path, package;
  msg_spec *type_spec;
  var_complex(const string &_type, const string &_name, msg_spec *_parent)
  : msg_var(_name, _parent), type_spec(NULL)
  {
    package = parent->package;
    vector<string> type_vec;
    string_split(_type, type_vec, "/");
    string spec_name;
    if (type_vec.size() >= 2)
    {
      package = type_vec[0];
      spec_name = type_vec[1];
    }
    else
      spec_name = type_vec[0];
    string spec_file = spec_name + string(".msg");
    if (spec_name == "Header")
      package = "roslib";
    if(_parent->is_root && !rospack_check_dep(g_pkg, package))
    {
      printf("Error: you're using package %s in a message/service, but you don't "
             "declare a dependency on that package.  Please add the missing "
             "dependency to your manifest.xml.\n", package.c_str());
      exit(13);
    }
    pkg_path = rospack_find(package);
    if (pkg_path.length() == 0)
    {
      printf("Error: couldn't find package %s which was referenced in the "
             "message.\n", package.c_str());
      exit(13);
    }
    spec_file = pkg_path + string("/msg/") + spec_file;
    //printf("spec file = [%s]\n", spec_file.c_str());
    type_spec_str = package + string(":") + spec_name;
    try
    {
      type_spec = new msg_spec(spec_file, package, spec_name, pkg_path);
    }
    catch(std::runtime_error err)
    {
      printf("Error: \"%s\" is neither a primitive type, nor a derived "
             "type that I can find.\n", spec_name.c_str());
      exit(13);
    }
    type_spec->class_name = package + string("::") + spec_name;
  }
  virtual string cpp_decl()
  {
    //return string("  ") + cpp_type_name() + string(" ") + name + string(";\n");
    string decl = string("(") + name +
            string("\n    :accessor ") + name + string("-val") +
            string("\n    :initarg :") + name +
            //string("\n    :type '") + type_spec->spec_name.c_str() +
            //string("\n    :initform NIL)");
            string("\n    :initform (make-instance '") + package +
            string("-msg:<") + type_spec->spec_name + string(">))");

    string export_decl = name + string("-val");
    for(unsigned int i=0;i<export_decl.size();i++)
      export_decl[i] = toupper(export_decl[i]);
    g_accessors.push_back(export_decl);
    return(decl);
  }
  virtual string test_populate(const string &prefix, int indent = 0)
  {
    return type_spec->test_populate(prefix + string(".") + name, indent);
  }
  virtual string cpp_type_name() { return type_spec->class_name; }
  virtual string equals(const string &prefix, int indent = 0)
  {
    if (prefix.length())
      return type_spec->equals(prefix + string(".") + name, indent);
    else
      return type_spec->equals(name, indent);
  }
  virtual string length_expr()
  {
    /*
    if (is_fixed_length())
      return type_spec->class_name +
             string("::serializationLength()");
    else
      */
      //return name + string(".serializationLength()");
    return string("(serialization-length ") + name + string(")");
  }
  virtual bool is_fixed_length()
  {
    return type_spec->is_fixed_length();
  }
  virtual vector<string> cpp_ctor_clauses() { return vector<string>(); }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> v;
    v.push_back(name + string("(copy.") + name + string(")"));
    return v;
  }
  virtual string cpp_copy_ctor_code() { return string(); }
  virtual string cpp_assign_code()
  {
    return string("    ") + name + string(" = copy.") + name + string(";\n");
  }
  virtual string cpp_dtor_code() { return string(); }
  virtual string helper_funcs() { return string(); }
  virtual string serialization_code()
  {
    //return string("    write_ptr = ") + name +
           //string(".serialize(write_ptr);\n");
    return string("(serialize ") + name + string(" ostream)");
  }
  virtual string deserialization_code()
  {
    //return string("    read_ptr = ") + name +
           //string(".deserialize(read_ptr);\n");
    return string("(deserialize ") + name + string(" istream)");
  }
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v)
  {
    vector<msg_spec *> types = type_spec->cpp_types(v);
    bool found = false;
    for (vector<msg_spec *>::iterator i = types.begin();
         !found && i != types.end(); ++i)
    {
      if ((*i)->spec_file == type_spec->spec_file)
        found = true;
    }
    if (!found)
      types.push_back(type_spec);
    return types;
  }
};

class var_array : public msg_var
{
public:
  string eletype;
  string lisp_eletype;
  int len;
  msg_var *ele_var;
  var_array(const string &_type, const string &_name, msg_spec *_parent)
  : msg_var(_name, _parent), len(0)
  {
    string::size_type open_bracket  = _type.find("[");
    string::size_type close_bracket = _type.find("]");
    eletype = _type.substr(0, open_bracket);

    lisp_eletype = eletype + ">";
    string::size_type slash = lisp_eletype.find("/");
    if (slash==string::npos) {
      lisp_eletype = "<" + lisp_eletype;
    }
    else {
      lisp_eletype.replace(slash, 1, "-msg:<");
    }

    if (close_bracket > open_bracket + 1)
    {
      string len_str = _type.substr(open_bracket+1,
                                    close_bracket - open_bracket - 1);
      len = atoi(len_str.c_str()); // todo: verify this string is all digits
    }
    ele_var = parent->make_var(eletype, "dummy");
  }
  virtual string cpp_decl()
  {
    /*
    const uint32_t CODE_LEN = 4096;
    char code[CODE_LEN];
    if (len)
      snprintf(code, CODE_LEN, "  %s %s[%d];\n",
               ele_var->cpp_type_name().c_str(), name.c_str(), len);
    else
      snprintf(code, CODE_LEN, "  %s *%s;\n" \
               "  uint32_t %s_size, %s_alloc_size;\n",
               ele_var->cpp_type_name().c_str(), name.c_str(),
               name.c_str(), name.c_str());
    return string(code);
    */
    ostringstream code;
    code << "(" << name << "\n    :accessor " << name << string("-val");
    code << "\n    :initarg :" << name;
    code << "\n    :initform (make-array " << len << " :initial-element ";
    if (is_integer(eletype) || eletype=="time" || eletype=="duration")
      code << "0";
    else if (is_float(eletype))
      code << "0.0";
    else if (is_bool(eletype))
      code << "nil";
    else if (eletype=="string")
      code << "\"\"";
    else
      code << "(make-instance '" << lisp_eletype << ")";

    code << "))";
      
    
    string export_decl = name + string("-val");
    for(unsigned int i=0;i<export_decl.size();i++)
      export_decl[i] = toupper(export_decl[i]);
    g_accessors.push_back(export_decl);
    return(code.str());
  }
  virtual string test_populate(const string &prefix, int indent = 0)
  {
    ostringstream code, iter_name_stream;
    string indent_str;
    for (int i = 0; i < indent; i++)
      indent_str += " ";
    iter_name_stream << name << indent << "_idx";
    string iter_name = iter_name_stream.str();
    if (len)
      code << indent_str << "  for (int " << iter_name << " = 0; "
           << iter_name << " < " << len << "; "
           << iter_name << "++)\n";
    else
    {
      code << indent_str << "  if (rand() % 2 != 0)\n";
      code << indent_str << "    " << prefix << ".set_" << name << "_size(rand() % 10000);\n";
      code << indent_str << "  for (int " << iter_name << " = 0; "
           << iter_name << " < " << prefix << "." << name << "_size; "
           << iter_name << "++)\n";
    }
    ele_var->name = name + string("[") + iter_name + string("]");
    code << indent_str << "  {\n";
    code << indent_str << ele_var->test_populate(prefix, indent+2);
    code << indent_str << "  }\n";
    return code.str();
  }
  virtual string equals(const string &prefix, int indent = 0)
  {
    ostringstream code, iter_name_stream;
    iter_name_stream << name << indent << "_idx";
    string iter_name = iter_name_stream.str();
    string indent_str;
    for (int i = 0; i < indent; i++)
      indent_str += " ";
    if (len)
      code << indent_str << "  for (int " << iter_name << " = 0; "
           << iter_name  << " < " << len << "; " << iter_name << "++)\n";
    else
      code << indent_str << "  for (int " << iter_name << " = 0; "
           << iter_name  << " < a." <<
              (prefix.length() ? prefix + string(".") : "") << name << "_size; "
           << iter_name << "++)\n";
    ele_var->name = name + string("[") + iter_name + string("]");
    code << indent_str << "  {\n";
    code << indent_str << ele_var->equals(prefix, indent+2);
    code << indent_str << "  }\n";
    return code.str();
  }
  virtual string cpp_type_name()
  {
    return ele_var->cpp_type_name() + " *";
  }
  virtual vector<string> cpp_ctor_clauses()
  {
    vector<string> v;
    if (!len)
    {
      v.push_back(name + string("(NULL)"));
      v.push_back(name + string("_size(0)"));
      v.push_back(name + string("_alloc_size(0)"));
    }
    return v;
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> v;
    if (!len)
    {
      v.push_back(name + string("(NULL)"));
      v.push_back(name + string("_size(0)"));
      v.push_back(name + string("_alloc_size(0)"));
    }
    return v;
  }
  virtual string cpp_copy_ctor_code()
  {
    ostringstream code;
    if (!len)
    {
      code << "    set_" << name << "_size(copy." << name << "_size);\n";
      code << "    for (size_t i = 0; i < " << name << "_size; i++)\n";
    }
    else
      code << "    for (size_t i = 0; i < " << len << "; i++)\n";
    code << "      " << name << "[i] = copy." << name << "[i];\n";
    return code.str();
  }
  virtual string cpp_assign_code()
  {
    return cpp_copy_ctor_code();
  }
  virtual string cpp_dtor_code()
  {
    ostringstream code;
    if (!len)
    {
      code << "    " << name << "_size = " << name << "_alloc_size = 0;\n"
           << "    if (" << name << ")\n"
           << "    {\n"
           << "      delete[] " << name << ";\n"
           << "      " << name << " = NULL;\n"
           << "    }\n";
    }
    return code.str();
  }
  virtual string length_expr()
  {
    uint32_t CODE_LEN = 4096;
    char code[CODE_LEN];
      //ele_var->name = name + string("[0]");
      ele_var->name = string("ele");
      //snprintf(code, CODE_LEN, "%s + (%s_size ? %s_size * %s : 0)",
               //(len ? "0" : "4"), name.c_str(), name.c_str(),
               //ele_var->length_expr().c_str());
      snprintf(code, CODE_LEN, "%s (reduce #'+ %s :key #'(lambda (ele) (declare (ignorable ele)) (+ %s)))",
               (len ? "0" : "4"),
               name.c_str(),
               ele_var->length_expr().c_str());
      ele_var->name = "dummy";

    /*
    else
    {
      snprintf(code, CODE_LEN, "%s + calc_%s_array_serialization_len()",
               (len ? "0" : "4"), name.c_str());
    }
    */
    return string(code);
  }
  virtual bool is_fixed_length()
  {
    return ((len && ele_var->is_fixed_length()) ? true : false);
  }
  virtual string helper_funcs()
  {
    ostringstream code;
    if (!len)
    {
      code << "  void set_" << name << "_size(uint32_t s)\n"
           << "  {\n    " << name << "_size = s;\n"
           << "    if (s <= " << name << "_alloc_size)\n      return;\n"
           << "    if (" << name << ")\n      delete[] " << name << ";\n"
           << "    if (s > 0)\n      " << name << " = new "
           << ele_var->cpp_type_name() << "[s];\n"
           << "    else\n      " << name << " = NULL;\n"
           << "    " << name << "_alloc_size = s;\n"
           << "  }\n";
      code << "  inline uint32_t get_" << name << "_size() const { return "
           << name << "_size; }\n";
    }
    else
    {
      char size_buf[100];
      snprintf(size_buf, sizeof(size_buf), "%d", len);
      code << "  inline uint32_t get_" << name << "_size() const { return "
           << size_buf << "; }\n";
    }
    if (!ele_var->is_fixed_length())
    {
      // e.g. arrays of strings, or arrays with structs with strings
      ele_var->name = name + string("[i]");
      code << "  uint32_t calc_" << name << "_array_serialization_len()\n"
           << "  {\n"
           << "    uint32_t l = 0;\n";
      if (!len)
        code << "    for (size_t i = 0; i < " << name << "_size; i++)\n";
      else
        code << "    for (size_t i = 0; i < " << len << "; i++)\n";
      code << "      l += " << ele_var->length_expr() << ";\n"
           << "    return l;\n  }\n";
    }
    // stl vector setters/getters
    code << "  void get_" << name << "_vec(std::vector<"
         << ele_var->cpp_type_name() << "> &v)\n"
         << "  {\n"
         << "    size_t vsize = ";
    if (len)
      code << len << ";\n";
    else
      code << "get_" << name << "_size();\n";
    code << "    v.clear();\n"
         << "    v.reserve(vsize);\n"
         << "    for (size_t i = 0; i < vsize; i++)\n"
         << "      v.push_back(" << name << "[i]);\n"
         << "  }\n";
    code << "  void set_" << name << "_vec(const std::vector<"
         << ele_var->cpp_type_name() << "> &v)\n"
         << "  {\n";
    if (!len)
    {
      code << "    set_" << name << "_size(v.size());\n"
           << "    for (size_t i = 0; i < v.size(); i++)\n";
    }
    else
    {
      code << "    for (size_t i = 0; i < v.size() && i < "
           << len << "; i++)\n";
    }
    code << "      " << name << "[i] = v[i];\n"
         << "  }\n";
    return code.str();
  }
  virtual string serialization_code()
  {
    // TODO: when handling arrays of fixed-length primitives, avoid
    // all the unnecessary function calls and blast the whole array
    // in with a memcpy
    ostringstream code;
    ele_var->name = "ele";

    // If length is known, just write the elements, else first write the length
    if (len)
    {
      code << "  (map nil #'(lambda (ele) "
           << ele_var->serialization_code() << ")" << name << ")";
    }
    
    else
    {
      //code << "    SROS_SERIALIZE_PRIMITIVE(write_ptr, " << name << "_size);\n"
           //<< "    for (size_t i = 0; i < " << name << "_size; i++)\n";
      ele_var->name = "ele";
      code << "(let ((__ros_arr_len (length " << name
           << ")))\n"
           << "    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)\n"
           << "    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)\n"
           << "    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)\n"
           << "    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))\n"
           << "  (map nil #'(lambda (ele) "
           << ele_var->serialization_code() << ")\n    " << name << ")";
      ele_var->name = "dummy";
    }

    


    /*
    else
    {
      code << "    for (size_t i = 0; i < " << len << "; i++)\n";
    }

    if(!msg_spec::is_primitive(eletype))
    {
      code << "      write_ptr = " << ele_var->name << ".serialize(write_ptr);\n";
    }
    else
    {
      code << "    {\n  " << ele_var->serialization_code() << "    }\n";
    }
    */

    return code.str();
  }
  virtual string deserialization_code()
  {
    // TODO: when handling arrays of fixed-length primitives, avoid
    // all the unnecessary function calls and blast the whole array
    // in with a memcpy
    ostringstream code;
    /*
    ele_var->name = name + string("[i]");
    if (!len)
      code << "    uint32_t __ros_" << name << "_len;\n"
           << "    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __ros_" << name << "_len);\n"
           << "    set_" << name << "_size(__ros_" << name << "_len);\n"
           << "    for (size_t i = 0; i < " << name << "_size; i++)\n";
    else
      code << "    for (size_t i = 0; i < " << len << "; i++)\n";

    if(!msg_spec::is_primitive(eletype))
      code << "      read_ptr = " << ele_var->name << ".deserialize(read_ptr);\n";
    else
      code << "    {\n  " << ele_var->deserialization_code() << "    }\n";

    return code.str();
    */
    ele_var->name = string("(aref vals i)");

    if (len) {
      code << "(setf " << name << " (make-array " << len << "))\n"
           << "  (let ((vals " << name << "))\n"
           << "    (dotimes (i " << len << ")\n";
      if( !msg_spec::is_primitive(eletype) ) {
        code << "    (setf (aref vals i) (make-instance '" << lisp_eletype << "))\n";
      }
      code << ele_var->deserialization_code() << "))";
    }
      
    else {
      code << "(let ((__ros_arr_len 0))\n"
           << "    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))\n"
           << "    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))\n"
           << "    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))\n"
           << "    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))\n"
           << "    (setf " << name << " (make-array __ros_arr_len))\n"
           << "    (let ((vals " << name << "))\n"
           << "      (dotimes (i __ros_arr_len)\n";
      if( !msg_spec::is_primitive(eletype) ) {
        code << "        (setf (aref vals i) (make-instance '" << lisp_eletype << "))\n";
      }    
      code << ele_var->deserialization_code() << ")))";
    }
    return code.str();
  }
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v)
  {
    return ele_var->cpp_types(v);
  }
};

class var_primitive : public msg_var
{
public:
  string type;
  var_primitive(const string &_type, const string &_name, msg_spec *_parent)
  : msg_var(_name, _parent), type(_type) { }
  virtual ~var_primitive() { }
  virtual string cpp_decl()
  {
    //return string("  ") + cpp_type_name() + string(" ") + name + string(";\n");
    string decl = string("(") + name +
            string("\n    :accessor ") + name + string("-val") +
            string("\n    :initarg :") + name;
    string export_decl = name + string("-val");
    for(unsigned int i=0;i<export_decl.size();i++)
      export_decl[i] = toupper(export_decl[i]);
    g_accessors.push_back(export_decl);
    if(is_integer(type))
      decl += string("\n    :initform 0");
    else if(is_bool(type))
      decl += string("\n    :initform nil");
    else if(is_float(type))
      decl += string("\n    :initform 0.0");
    else if(type == "string")
      decl += string("\n    :initform \"\"");
    else if(type == "time" || type == "duration")
      decl += string("\n    :initform 0");
    decl += string(")");
    return decl;
  }
  virtual string cpp_type_name()
  {
    if (type == "char" || type == "uint8" || type=="bool")
      return "uint8_t";
    else if (type == "byte" || type == "int8")
      return "int8_t";
    else if (type == "uint16")
      return "uint16_t";
    else if (type == "int16")
      return "int16_t";
    else if (type == "uint32")
      return "uint32_t";
    else if (type == "int32")
      return "int32_t";
    else if (type == "uint64")
      return "uint64_t";
    else if (type == "int64")
      return "int64_t";
    else if (type == "float32")
      return "float";
    else if (type == "float64")
      return "double";
    else if (type == "string")
      return "std::string";
    else if (type == "time")
      return "ros::Time";
    else if (type == "duration")
      return "ros::Duration";
    else
      return string("\n#error woah! unhandled primitive type ") + type +
             string("\n");
  }
  virtual string length_expr()
  {
    if (type == "byte" || type == "char" || type == "uint8" || type == "int8" || type=="bool")
      return "1";
    else if (type == "uint16" || type == "int16")
      return "2";
    else if (type == "uint32" || type == "int32")
      return "4";
    else if (type == "uint64" || type == "int64")
      return "8";
    else if (type == "float32")
      return "4";
    else if (type == "float64")
      return "8";
    else if (type == "time" || type == "duration")
      return "8";
    else if (type == "string")
      return string("4 ") + string("(length ") + name +
              string(")");
    else
      return "\n#error woah! bogus length_expr in var_primitive\n";
  }
  virtual bool is_fixed_length()
  {
    if (type == "string")
      return false;
    else
      return true;
  }
  virtual vector<string> cpp_ctor_clauses()
  {
    if (type == "string" || type == "time" || type == "duration")
      return vector<string>();
    vector<string> c;
    c.push_back(name + "(0)");
    return c;
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> v;
    v.push_back(name + string("(copy.") + name + string(")"));
    return v;
  }
  virtual string cpp_copy_ctor_code() { return string(); }
  virtual string cpp_assign_code()
  {
    return string("    ") + name + string(" = copy.") + name + string(";\n");
  }
  virtual string cpp_dtor_code() { return string(); }
  virtual string test_populate(const string &prefix, int indent = 0)
  {
    string indent_str;
    for (int i = 0; i < indent + 2; i++)
      indent_str += " ";

    if (type == "byte"    || type == "char"  ||
        type == "uint8"   || type == "int8"  ||
        type == "uint16"  || type == "int16" ||
        type == "uint32"  || type == "int32" ||
        type == "uint64"  || type == "int64" ||
        // type == "bool" || 
        // Commenting out because I don't think this will actually be called
        type == "float32" || type == "float64")
      return indent_str + prefix + string(".") + name + string(" = rand();\n");
    else if (type == "time" || type == "duration")
      return indent_str + prefix + "." + name + string(".sec = rand();\n") +
             indent_str + prefix + "." + name + string(".nsec = rand();\n");
    else if (type == "string")
      return indent_str + string("if (rand() % 2 != 0)\n") +
             indent_str + "  " + prefix + string(".") + name +
             string(" = \"blahblahblah\";\n");
    else
      return string("\n#error woah! bogus primitive type\n");
  }
  virtual string equals(const string &prefix, int indent = 0)
  {
    // don't compare the header timestamps, since they change
    // each time it's serialized
    if (name == "stamp")
      return string();
    string indent_str;
    for (int i = 0; i < indent; i++)
      indent_str += " ";
    if (prefix.length())
      return indent_str + string("  ok &= (") +
             string("a.") + prefix + string(".") + name + string(" == ") +
             string("b.") + prefix + string(".") + name + string(");\n");
    else
      return indent_str + string("  ok &= (a.") + name + string(" == ") +
             string("b.") + name + string(");\n");
  }
  virtual string helper_funcs()
  {
    return string();
  }
  virtual string serialization_code()
  {
    const uint32_t CODE_LEN = 4096;
    char code[CODE_LEN];
    if (type == "string")
    {
      /*
      string safe_name = name;
      for (int i = 0; i < safe_name.length(); i++)
        if (safe_name[i] == '[' || safe_name[i] == ']')
          safe_name[i] = '_';
      snprintf(code, CODE_LEN,
        "    unsigned __ros_%s_len = %s.length();\n" \
        "    SROS_SERIALIZE_PRIMITIVE(write_ptr, __ros_%s_len);\n" \
        "    SROS_SERIALIZE_BUFFER(write_ptr, %s.c_str(), __ros_%s_len);\n",
        safe_name.c_str(), name.c_str(), safe_name.c_str(),
        name.c_str(), safe_name.c_str());
        */
      snprintf(code, CODE_LEN,
               "(let ((__ros_str_len (length %s)))\n"
               "    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)\n"
               "    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)\n"
               "    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)\n"
               "    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))\n"
               "  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) %s)",
               name.c_str(), name.c_str());
    }
    else if (type == "time" || type == "duration")
    {
      /*
      snprintf(code, CODE_LEN,
        "    SROS_SERIALIZE_PRIMITIVE(write_ptr, %s.sec);\n"
        "    SROS_SERIALIZE_PRIMITIVE(write_ptr, %s.nsec);\n",
        name.c_str(), name.c_str());
        */
      snprintf(code, CODE_LEN,
               "(let ((__sec (floor %s))\n"
               "        (__nsec (round (* 1e9 (- %s (floor %s))))))\n"
               "    (write-byte (ldb (byte 8 0) __sec) ostream)\n"
               "    (write-byte (ldb (byte 8 8) __sec) ostream)\n"
               "    (write-byte (ldb (byte 8 16) __sec) ostream)\n"
               "    (write-byte (ldb (byte 8 24) __sec) ostream)\n"
               "    (write-byte (ldb (byte 8 0) __nsec) ostream)\n"
               "    (write-byte (ldb (byte 8 8) __nsec) ostream)\n"
               "    (write-byte (ldb (byte 8 16) __nsec) ostream)\n"
               "    (write-byte (ldb (byte 8 24) __nsec) ostream))",
               name.c_str(), name.c_str(), name.c_str());
    }
    else
    {
      //snprintf(code, CODE_LEN,
        //"    SROS_SERIALIZE_PRIMITIVE(write_ptr, %s);\n", name.c_str());

      if (type == "char" || type == "uint8")
        snprintf(code, CODE_LEN, 
                 "  (write-byte (ldb (byte 8 0) %s) ostream)",
                 name.c_str());
      else if (type == "bool")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) (if %s 1 0)) ostream)",
                 name.c_str());
      else if (type == "byte" || type == "int8")
        snprintf(code, CODE_LEN, 
                 "  (write-byte (ldb (byte 8 0) %s) ostream)",
                 name.c_str());
      else if (type == "uint16")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)",
                 name.c_str(), name.c_str());
      else if (type == "int16")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)",
                 name.c_str(), name.c_str());
      else if (type == "uint32")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 16) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 24) %s) ostream)",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "int32")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 16) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 24) %s) ostream)",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "uint64")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 16) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 24) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 32) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 40) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 48) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 56) %s) ostream)",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str(),
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "int64")
        snprintf(code, CODE_LEN,
                 "  (write-byte (ldb (byte 8 0) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 8) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 16) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 24) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 32) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 40) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 48) %s) ostream)\n"
                 "  (write-byte (ldb (byte 8 56) %s) ostream)",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str(),
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "float32")
      {
        snprintf(code, CODE_LEN,
                 "(let ((bits (roslisp-utils:encode-single-float-bits %s)))\n"
                 "    (write-byte (ldb (byte 8 0) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 8) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 16) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 24) bits) ostream))",
                 name.c_str());
      }
      else if (type == "float64")
        snprintf(code, CODE_LEN,
                 "(let ((bits (roslisp-utils:encode-double-float-bits %s)))\n"
                 "    (write-byte (ldb (byte 8 0) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 8) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 16) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 24) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 32) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 40) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 48) bits) ostream)\n"
                 "    (write-byte (ldb (byte 8 56) bits) ostream))",
                 name.c_str());
    }

    return string(code);
  }
  virtual string deserialization_code()
  {
    const uint32_t CODE_LEN = 4096;
    char code[CODE_LEN];
    if (type == "string")
    {
      /*
      string safe_name = name;
      for (int i = 0; i < safe_name.length(); i++)
        if (safe_name[i] == '[' || safe_name[i] == ']')
          safe_name[i] = '_';

      snprintf(code, CODE_LEN,
        "    unsigned __ros_%s_len;\n" \
        "    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __ros_%s_len);\n" \
        "    %s = std::string((const char *)read_ptr, __ros_%s_len);\n" \
        "    read_ptr += __ros_%s_len;\n",
        safe_name.c_str(), safe_name.c_str(), name.c_str(),
        safe_name.c_str(), safe_name.c_str());
        */
      snprintf(code, CODE_LEN,
               "(let ((__ros_str_len 0))\n"
               "    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))\n"
               "    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))\n"
               "    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))\n"
               "    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))\n"
               "    (setf %s (make-string __ros_str_len))\n"
               "    (dotimes (__ros_str_idx __ros_str_len msg)\n"
               "      (setf (char %s __ros_str_idx) (code-char (read-byte istream)))))",
               name.c_str(), name.c_str());

    }
    else if (type == "time" || type == "duration")
    {
      //snprintf(code, CODE_LEN,
        //"    SROS_DESERIALIZE_PRIMITIVE(read_ptr, %s.sec);\n"
        //"    SROS_DESERIALIZE_PRIMITIVE(read_ptr, %s.nsec);\n",
        //name.c_str(), name.c_str());
      snprintf(code, CODE_LEN,
               "(let ((__sec 0) (__nsec 0))\n"
               "    (setf (ldb (byte 8 0) __sec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 8) __sec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 16) __sec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 24) __sec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 0) __nsec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 8) __nsec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 16) __nsec) (read-byte istream))\n"
               "    (setf (ldb (byte 8 24) __nsec) (read-byte istream))\n"
               "    (setf %s (+ __sec (/ __nsec 1e9))))",
               name.c_str());
    }
    else
    {
      //snprintf(code, CODE_LEN,
        //"    sros_deserialize_primitive(read_ptr, %s);\n", name.c_str());


      if (type == "char" || type == "uint8")
        snprintf(code, CODE_LEN, 
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))",
                 name.c_str());
      else if (type == "byte" || type == "int8")
        snprintf(code, CODE_LEN, 
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))",
                 name.c_str());
      else if (type == "bool" || type == "bool")
        snprintf(code, CODE_LEN,
                 "(setf %s (not (zerop (read-byte istream))))",
                 name.c_str());
      else if (type == "uint16")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))",
                 name.c_str(), name.c_str());
      else if (type == "int16")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))",
                 name.c_str(), name.c_str());
      else if (type == "uint32")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 16) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 24) %s) (read-byte istream))",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "int32")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 16) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 24) %s) (read-byte istream))",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "uint64")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 16) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 24) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 32) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 40) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 48) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 56) %s) (read-byte istream))",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str(),
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "int64")
        snprintf(code, CODE_LEN,
                 "(setf (ldb (byte 8 0) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 8) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 16) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 24) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 32) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 40) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 48) %s) (read-byte istream))\n"
                 "  (setf (ldb (byte 8 56) %s) (read-byte istream))",
                 name.c_str(), name.c_str(), name.c_str(), name.c_str(),
                 name.c_str(), name.c_str(), name.c_str(), name.c_str());
      else if (type == "float32")
        snprintf(code, CODE_LEN,
                 "(let ((bits 0))\n"
                 "    (setf (ldb (byte 8 0) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 8) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 16) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 24) bits) (read-byte istream))\n"
                 "    (setf %s (roslisp-utils:decode-single-float-bits bits)))",
                 name.c_str());
      else if (type == "float64")
        snprintf(code, CODE_LEN,
                 "(let ((bits 0))\n"
                 "    (setf (ldb (byte 8 0) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 8) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 16) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 24) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 32) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 40) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 48) bits) (read-byte istream))\n"
                 "    (setf (ldb (byte 8 56) bits) (read-byte istream))\n"
                 "    (setf %s (roslisp-utils:decode-double-float-bits bits)))",
                 name.c_str());
      /*
       * todo: handle these types
      else if (type == "string")
        return "std::string";
      else if (type == "time")
        return "ros::time";
      else if (type == "duration")
        return "ros::duration";
        */
    }
    return string(code);
  }
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v)
  {
    return v;
  }
};

class var_constant
{
public:
  string name, type, constant;
  var_constant(const string &_type, const string &_name, 
                const string &_constant)
  : name(_name), type(_type), constant(_constant) { }
  virtual ~var_constant() { }
};

msg_spec::msg_spec(const string &_spec_file, const string &_package,
                   const string &_spec_name, const string &_pkg_path,
                   bool is_filename, bool _is_root)
: spec_file(_spec_file), spec_name(_spec_name), package(_package),
  pkg_path(_pkg_path), is_root(_is_root)
{
  if (is_filename)
  {
    // parse it
    FILE *f = fopen(spec_file.c_str(), "r");
    if (!f)
      throw std::runtime_error("couldn't open spec file");

    if (is_root)
    {
      {
        // compute md5sum
        string cmd = string("`rospack find roslib`/scripts/gendeps --md5 ") + spec_file;
        FILE *md5pipe = popen(cmd.c_str(), "r");
        if (!md5pipe)
          throw std::runtime_error("couldn't launch md5sum in genmsg_cpp\n");
        char md5buf[PATH_MAX];
        if (!fgets(md5buf, PATH_MAX, md5pipe))
          throw std::runtime_error("couldn't read md5sum pipe in genmsg_cpp\n");
        char *md5str = strtok(md5buf, " \t\n");
        md5sum = string(md5str);
        // call pclose sometime
      }

      {
        std::stringstream ss;
        // compute concatenated definition
        string cmd = string("`rospack find roslib`/scripts/gendeps --cat ") + spec_file;
        FILE *catpipe = popen(cmd.c_str(), "r");
        if (!catpipe)
          throw std::runtime_error("couldn't launch gendeps in genmsg_cpp\n");
        char buf[1024];
        while (fgets(buf, 1024, catpipe))
        {
          std::string str(buf, 1024);
          ss << buf;
        }
        // call pclose sometime

        full_definition = ss.str();
      }
    }

    const int LINEBUF_LEN = 1024;
    char linebuf[LINEBUF_LEN];

    for (int linenum = 1; !feof(f); linenum++)
    {
      if (!fgets(linebuf, LINEBUF_LEN, f))
        break; // hit EOF
      process_line(linebuf, linenum); // no need for error checking b/c
                                      // they are all fatal
    }
    fclose(f);
  }
  else
  {
    vector<string> lines;
    string_split(string(spec_file), lines, "\n");
    for (size_t i = 0; i < lines.size(); i++)
    {
      lines[i] += "\n";
      char linebuf[1024];
      strncpy(linebuf, lines[i].c_str(), sizeof(linebuf)-1);
      process_line(linebuf, i+1);
      //printf("line = [%s]\n", lines[i].c_str());
    }
  }
}

/*
bool msg_spec::process_line(char *linebuf, int linenum)
{
  if (strlen(linebuf) == 0 || linebuf[0] == '#')
    return true;
  // see if the first non-whitespace character is a hash
  size_t i;
  for (i = 0; i < strlen(linebuf); i++)
    if (linebuf[i] != ' ' && linebuf[i] != '\t')
      break;
  if (i == strlen(linebuf) || linebuf[i] == '\n')
    return true; // this line was empty.
  if (linebuf[i] == '#')
    return true; // this line was a comment.
  char *token = strtok(linebuf, " \n");
  if (!token) // this should never happen due to checks above
  {
    char err_msg[1024];
    snprintf(err_msg, 1024, "couldn't parse a token out of spec file %s " \
             " on line %d", spec_file.c_str(), linenum);
    throw std::runtime_error(string(err_msg));
  }
  string type = string(token);
  token = strtok(NULL, " \n");
  if (!token)
  {
    printf("woah! on line %d of %s, there was only one token.\n",
           linenum, spec_file.c_str());
    printf("each line needs to have two tokens: the first is the type\n" \
           "and the second is the variable name.\n");
    exit(6);
  }
  string name = string(token);
  vars.push_back(make_var(type, name));
  return true;
}
*/

static string trim_whitespace(string str, const char *extra_ws_chars = "")
{
  string ws = " \t\r\n";
  ws += extra_ws_chars;
  size_t start = str.find_first_not_of(ws);
  if (start == string::npos)
    return string(); // nothing there but whitespace
  size_t end = str.find_first_of(ws, start);
  return str.substr(start, end - start);
}

bool msg_spec::process_line(char *linebuf, int linenum)
{
  if (strlen(linebuf) == 0 || linebuf[0] == '#')
    return true;
  // see if the first non-whitespace character is a hash
  size_t i;
  for (i = 0; i < strlen(linebuf); i++)
    if (linebuf[i] != ' ' && linebuf[i] != '\t')
      break;
  if (i == strlen(linebuf) || linebuf[i] == '\n')
    return true; // this line was empty.
  if (linebuf[i] == '#')
    return true; // this line was a comment.
  string linecopy(linebuf);
  char *token = strtok(linebuf, " \n\t");
  if (!token) // this should never happen due to checks above
  {
    char err_msg[1024];
    snprintf(err_msg, 1024, "couldn't parse a token out of spec file %s " \
             " on line %d", spec_file.c_str(), linenum);
    throw std::runtime_error(string(err_msg));
  }
  string name, type = string(token);
  bool constant_allowed = is_primitive(type) &&
                          type != string("time") &&
                          type != string("duration");
  size_t hash_pos   = linecopy.find('#');
  size_t equals_pos = linecopy.find('=');
  token = strtok(NULL, " \n\t=");
  if (!token)
  {
    printf("woah! on line %d of %s, there was only one token.\n",
           linenum, spec_file.c_str());
    printf("each line needs to have two tokens: the first is the type\n" \
           "and the second is the variable name.\n");
    exit(6);
  }
  name = string(token);
  // ignore the equals sign if it's after a hash
  if (equals_pos == string::npos ||
      (hash_pos != string::npos && equals_pos > hash_pos))
  {
    // this was a variable declaration. nothing more to parse; get out of here.
    vars.push_back(make_var(type, name));
    return true;
  }
  if (!constant_allowed)
  {
    printf("woah! constants are only allowed on primitive numeric types and "
           "strings.\n");
    exit(7);
  }
  // there is an equals sign in here, so it must be a constant declaration
  string constant = linecopy.substr(equals_pos+1);
  for (size_t line_terminator = constant.find_first_of("\r\n");
       line_terminator != string::npos;
       line_terminator = constant.find_first_of("\r\n"))
    constant = constant.erase(line_terminator, 1);
  // if it's not a string constant, trim the whitespace after the first token
  if (type != "string")
  {
    constant = trim_whitespace(constant, "#");
    //printf("nonstring constant = [%s]\n", constant.c_str());
  }
  constant_vars.push_back(new var_constant(type, name, constant));
  return true;
}

// TODO: migrate this into msg_spec
static bool is_integer(const string &type)
{
  vector<string> prims;
  prims.push_back("byte");
  prims.push_back("char");
  prims.push_back("uint8");
  prims.push_back("int8");
  prims.push_back("uint16");
  prims.push_back("int16");
  prims.push_back("uint32");
  prims.push_back("int32");
  prims.push_back("uint64");
  prims.push_back("int64");
  for (vector<string>::iterator i = prims.begin(); i != prims.end(); ++i)
    if (*i == type)
      return true;
  return false;
}

static bool is_bool(const string& type)
{
  return (type=="bool");
}    
  

// TODO: migrate this into msg_spec
static bool is_float(const string &type)
{
  vector<string> prims;
  prims.push_back("float32");
  prims.push_back("float64");
  for (vector<string>::iterator i = prims.begin(); i != prims.end(); ++i)
    if (*i == type)
      return true;
  return false;
}

bool msg_spec::is_primitive(const string &type)
{
  vector<string> prims;
  prims.push_back("byte");
  prims.push_back("char");
  prims.push_back("uint8");
  prims.push_back("int8");
  prims.push_back("bool");
  prims.push_back("uint16");
  prims.push_back("int16");
  prims.push_back("uint32");
  prims.push_back("int32");
  prims.push_back("uint64");
  prims.push_back("int64");
  prims.push_back("float32");
  prims.push_back("float64");
  prims.push_back("string");
  prims.push_back("time");
  prims.push_back("duration");
  for (vector<string>::iterator i = prims.begin(); i != prims.end(); ++i)
    if (*i == type)
      return true;
  return false;
}

bool msg_spec::is_array(const string &type)
{
  if (string::npos != type.find('['))
    return true;
  else
    return false;
}

bool g_loaded_package_lisp;

void msg_spec::emit_cpp_includes(FILE *f)
{
  assert(f);
  if(!g_loaded_package_lisp)
  {
    fprintf(f, "; Auto-generated. Do not edit!\n\n");
    // fprintf(f, "(roslisp:load-if-necessary \"%s/lisp/%s/_package.lisp\")\n", g_path.c_str(), g_pkg.c_str());
    g_loaded_package_lisp = true;
  }
  vector<msg_spec *> sub_specs;
  sub_specs = cpp_types(sub_specs);

  char fname[PATH_MAX];
  snprintf(fname, PATH_MAX, "%s/lisp/%s/.%s.asd-dep", g_path.c_str(), g_pkg.c_str(), spec_name.c_str());

  set<string> deps;
  read_depfile( fname, deps );
  for (vector<msg_spec *>::iterator i = sub_specs.begin();
         i != sub_specs.end(); ++i)
  {
      deps.insert( (*i)->package + string("-msg") );
  }
  write_depfile( fname, deps );
}

void msg_spec::emit_cpp_class(FILE *f, bool for_srv, const string &srv_name)
{
  assert(f);
  if (!for_srv)
  {
    emit_cpp_includes(f);
    //fprintf(f, "namespace %s\n{\n\n", package.c_str());
    fprintf(f, "\n");
    fprintf(f, "(in-package %s-msg)\n\n", g_pkg.c_str());
    fprintf(f, "\n");
  }

  //fprintf(f, "//! \\htmlinclude %s.msg.html\n\n",g_name.c_str());
  fprintf(f, ";//! \\htmlinclude %s.msg.html\n\n",g_name.c_str());
  //fprintf(f, "class %s : public ros::msg\n{\npublic:\n", g_name.c_str());
  // TODO: inherit from superclass?
  fprintf(f, "(defclass <%s> (ros-message)\n  (", g_name.c_str());

  bool header_present = false;
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    // pretty-printing
    if(v != vars.begin())
      fprintf(f, "\n   ");
    fprintf(f, "%s", (*v)->cpp_decl().c_str());
    if ((*v)->name == "header" && ((*v)->cpp_type_name().find("Header") != string::npos))
      header_present = true;
  }
  fprintf(f, ")\n");
  fprintf(f, ")\n");

  if(!constant_vars.empty())
  {
    fprintf(f, "(defmethod symbol-codes ((msg-type (eql '<%s>)))\n"
            "  \"Constants for message type '<%s>\"\n",
            g_name.c_str(), g_name.c_str());
    for(unsigned int i=0;i<constant_vars.size();i++)
    {
      fprintf(f, "%s(:%s . %s)%s\n",
              (i == 0) ? "  '(" : "    ",
              constant_vars[i]->name.c_str(),
              constant_vars[i]->constant.c_str(),
              (i == (constant_vars.size() - 1)) ? ")" : "");
    }
    fprintf(f, ")\n");
  }

  fprintf(f, "(defmethod serialize ((msg <%s>) ostream)\n"
             "  \"Serializes a message object of type '<%s>\"\n",
             g_name.c_str(), g_name.c_str());
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    string oldname = (*v)->name;
    (*v)->name = string("(slot-value msg '") + (*v)->name + string(")");
    fprintf(f,"  %s\n", (*v)->serialization_code().c_str());
    (*v)->name = oldname;
  }
  fprintf(f,")\n");

  fprintf(f, "(defmethod deserialize ((msg <%s>) istream)\n"
             "  \"Deserializes a message object of type '<%s>\"\n",
             g_name.c_str(), g_name.c_str());
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    string oldname = (*v)->name;
    (*v)->name = string("(slot-value msg '") + (*v)->name + string(")");
    fprintf(f,"  %s\n", (*v)->deserialization_code().c_str());
    (*v)->name = oldname;
  }
  fprintf(f,"  msg\n");
  fprintf(f,")\n");

  if(!for_srv)
    fprintf(f, "(defmethod ros-datatype ((msg (eql '<%s>)))\n"
            "  \"Returns string type for a message object of type '<%s>\"\n"
            "  \"%s/%s\")\n",
            g_name.c_str(), g_name.c_str(), 
            g_pkg.c_str(), g_name.c_str());
  else
    fprintf(f, "(defmethod ros-datatype ((msg (eql '<%s>)))\n"
            "  \"Returns string type for a service object of type '<%s>\"\n"
            "  \"%s/%s\")\n",
            g_name.c_str(),
            g_name.c_str(), 
            g_pkg.c_str(), srv_name.c_str());

  if (server_md5sum.length())
  {
    fprintf(f, "(defmethod md5sum ((type (eql '<%s>)))\n"
            "  \"Returns md5sum for a message object of type '<%s>\"\n"
            "  #x%s)\n",
            g_name.c_str(), g_name.c_str(), server_md5sum.c_str());
  }
  else
  {
    fprintf(f, "(defmethod md5sum ((type (eql '<%s>)))\n"
            "  \"Returns md5sum for a message object of type '<%s>\"\n"
            "  #x%s)\n",
            g_name.c_str(), g_name.c_str(), md5sum.c_str());
  }

  fprintf(f, 
          "(defmethod message-definition ((type (eql '<%s>)))\n"
          "  \"Returns full string definition for message of type '<%s>\"\n"
          "  (format nil \"",
          g_name.c_str(), g_name.c_str());
  vector<string> definition_lines;
  string_split(full_definition, definition_lines, "\n");
  vector<string>::iterator it = definition_lines.begin();
  vector<string>::iterator end = definition_lines.end();
  for (; it != end; ++it)
  {
    std::string& line = *it;

    // Escape bare \ and "
    size_t pos = line.find("\\");
    while (pos != std::string::npos)
    {
      line.insert(pos, "\\");

      pos = line.find("\\", pos + 2);
    }

    pos = line.find("\"");
    while (pos != std::string::npos)
    {
      line.insert(pos, "\\");

      pos = line.find("\"", pos + 2);
    }

    //fprintf(f, "    \"%s\\n\"\n", line.c_str());
    fprintf(f, "%s~%%", line.c_str());
  }
  fprintf(f, "\"))\n");

  fputs(serializationLength_func().c_str(), f);
  fputs(to_list_func().c_str(), f);

  string f2_basename;

  if(for_srv)
  {
    if(g_name.substr(g_name.size()-9) == string("-response"))
      f2_basename = g_name.substr(0, g_name.size()-9);
    else
      return;
  }
  else
  {
    f2_basename = g_name;
  }

  string f2name = g_path + string("/lisp/") + g_pkg + string("/_package_") + f2_basename + string(".lisp");
  FILE *f2 = fopen(f2name.c_str(), "w");
  if (!f2)
    throw std::runtime_error("couldn't write to file");
  if (for_srv)
  {
    fprintf(f2, "(in-package %s-srv)\n", g_pkg.c_str());
  }
  else 
  {
    fprintf(f2, "(in-package %s-msg)\n", g_pkg.c_str());
  }
  fprintf(f2, "(export '(");
  for(list<string>::iterator it = g_accessors.begin();
      it != g_accessors.end();
      ++it)
  {
    if(it != g_accessors.begin())
      fprintf(f2, "          ");
    fprintf(f2, "%s\n", (*it).c_str());
  }
  fprintf(f2, "))\n");
  fclose(f2);
}

string msg_spec::serializationLength_func()
{
  string s;
//  if (!is_fixed_length())
    //s += "  inline uint32_t serializationLength()\n  {\n";
  s += string("(defmethod serialization-length ((msg <") + g_name +
          string(">))\n");
//  else
//    s += "  static uint32_t s_serializationLength()\n  {\n";
  //s += "    unsigned l = 0;\n";
  s += "  (+ 0\n";
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    string oldname = (*v)->name;
    (*v)->name = string("(slot-value msg '") + (*v)->name + string(")");
    // Is it an array of non-primitives?
    // If so, then we need declare the dummy variable that
    // will be used to determine serialization length
    /*
    var_array* array_ptr = dynamic_cast<var_array*>(*v);
    if(array_ptr && !msg_spec::is_primitive(array_ptr->eletype))
      s += string("    {\n    ") + array_ptr->ele_var->cpp_decl() +
              string("  ");
    */
    //s += string("    l += ") + (*v)->length_expr() + string("; // ") +
      //(*v)->name + string("\n");
    s += string("     ") + (*v)->length_expr() + string("\n");
    /*
    if(array_ptr && !msg_spec::is_primitive(array_ptr->eletype))
      s += string("    }\n");
    */
    (*v)->name = oldname;
  }
  //s += "    return l;\n";
  //s += "  }\n";
  s += "))\n";
  return s;
}

string msg_spec::to_list_func()
{
  string s;
  s += string("(defmethod ros-message-to-list ((msg <") + g_name +
          string(">))\n");
  s += string("  \"Converts a ROS message object to a list\"\n");
  s += string("  (list '<") + g_name + string(">\n");
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    s += string("    (cons ':") + (*v)->name + string(" (") + (*v)->name + string("-val") + string(" msg))\n");
  s += "))\n";
  return s;
}

bool msg_spec::is_fixed_length()
{
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    if (!(*v)->is_fixed_length())
      return false;
  return true;
}

vector<msg_spec *> msg_spec::cpp_types(vector<msg_spec *> types)
{
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    types = (*v)->cpp_types(types);
  return types;
}

string msg_spec::test_populate(const string &prefix, int indent)
{
  ostringstream code;
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    code << (*v)->test_populate(prefix, indent);
  return code.str();
}

string msg_spec::equals(const string &prefix, int indent)
{
  ostringstream code;
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    code << (*v)->equals(prefix, indent);
  return code.str();
}

msg_var *msg_spec::make_var(const string &type, const string &name)
{
  if (msg_spec::is_primitive(type))
    return new var_primitive(type, name, this);
  else if (msg_spec::is_array(type))
    return new var_array(type, name, this);
  else
    return new var_complex(type, name, this);
}

// Read and write dependencies file
void write_depfile( const char *filename, const set<string> &deps )
{
  FILE *dependencies_file;

  dependencies_file = fopen(filename, "w");
  if (!dependencies_file)
  {
      printf("woah! couldn't write to %s\n", filename);
      exit(7);
  }
  for( set<string>::iterator it=deps.begin();
       it != deps.end();
       it++ )
  {
      fprintf(dependencies_file, "%s\n", it->c_str() );
  }
  fclose( dependencies_file );
}

set<string> &read_depfile( const char *filename, set<string> &deps )
{
  FILE *dependencies_file;
  const int LINEBUF_LEN = 1024;
  char linebuf[LINEBUF_LEN];

  dependencies_file = fopen(filename, "r");
  if (!dependencies_file)
  {
      // Do nothing if file already exists.
      return deps;
  }
  while( !feof( dependencies_file )
      && fgets( linebuf, LINEBUF_LEN, dependencies_file ) )
  {
      deps.insert( string( linebuf ) );
  }
  return deps;
}
