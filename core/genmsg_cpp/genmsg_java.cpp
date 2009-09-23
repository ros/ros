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
#include <map>
#include <cstdio>
#include <cerrno>
#include <cassert>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/param.h>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include "msgspec.h"
#include "utils.h"

using namespace std;

string primitive_type(const string &type) {
	map<string, string> m;
	m["byte"] = "byte";
	m["char"] = "byte";
	m["uint8"] = "byte";
	m["int8"] = "byte";
	m["uint16"] = "short";
	m["int16"] = "short";
	m["uint32"] = "int";
	m["int32"] = "int";
	m["uint64"] = "long";
	m["int64"] = "long";
	m["float32"] = "float";
	m["float64"] = "double";
	m["string"] = "java.lang.String";
	m["time"] = "ros.communication.Time";
	m["duration"] = "ros.communication.Duration";

	map<string,string>::iterator it = m.find(type);

	if (it == m.end()) {
		printf("woah! unknown primitive type: [%s]\n", type.c_str());
		exit(5);
	}

	return it->second;
}

string nonprimitive_type(string package, const string type) {
	for(unsigned int i = 0; i < package.size(); i++) {
		if (package[i] == '/') package[i] = '.';
	}
	return "ros.pkg." + package + ".msg." + type;
}
string primitive_type_cap(const string &type) {
	map<string, string> m;
	m["byte"] = "Byte";
	m["char"] = "Byte";
	m["uint8"] = "Byte";
	m["int8"] = "Byte";
	m["uint16"] = "Short";
	m["int16"] = "Short";
	m["uint32"] = "Int";
	m["int32"] = "Int";
	m["uint64"] = "Long";
	m["int64"] = "Long";
	m["float32"] = "Float";
	m["float64"] = "Double";
	m["string"] = "String";
	m["time"] = "Time";
	m["duration"] = "Duration";

	map<string,string>::iterator it = m.find(type);

	if (it == m.end()) {
		printf("woah!  unknown primitive type: [%s]\n", type.c_str());
		exit(5);
	}

	return it->second;
}


string primitive_type_box(const string &type) {
	map<string, string> m;
	m["byte"] = "java.lang.Byte";
	m["char"] = "java.lang.Byte";
	m["uint8"] = "java.lang.Byte";
	m["int8"] = "java.lang.Byte";
	m["uint16"] = "java.lang.Short";
	m["int16"] = "java.lang.Short";
	m["uint32"] = "java.lang.Integer";
	m["int32"] = "java.lang.Integer";
	m["uint64"] = "java.lang.Long";
	m["int64"] = "java.lang.Long";
	m["float32"] = "java.lang.Float";
	m["float64"] = "java.lang.Double";
	m["string"] = "java.lang.String";
	m["time"] = "ros.communication.Time";
	m["duration"] = "ros.communication.Duration";

	map<string,string>::iterator it = m.find(type);

	if (it == m.end()) {
		printf("woah!   unknown primitive type: [%s]\n", type.c_str());
		exit(5);
	}

	return it->second;
}


bool is_number(const std::string &type)
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
  prims.push_back("float32");
  prims.push_back("float64");
  for (vector<string>::iterator i = prims.begin(); i != prims.end(); ++i)
    if (*i == type)
      return true;
  return false;
}

int number_bytes (const std::string &type) {
	string typec = primitive_type_cap(type);
	if (typec == "Byte") return 1;
	if (typec == "Short") return 2;
	if (typec == "Int") return 4;
	if (typec == "Long") return 8;
	if (typec == "Float") return 4;
	if (typec == "Double") return 8;

	printf("woah!   unknown primitive type: [%s]\n", type.c_str());
	exit(5);
	return 0;
}

//For now, forget about equals and hashcode.

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
  virtual string cpp_ctor_code() { return string(); }
  virtual string cpp_assign_code() = 0;
  virtual string cpp_dtor_code() = 0; // Repurpose for hash_code method?
  virtual string helper_funcs() = 0;
  virtual string serialization_code() = 0;
  virtual string deserialization_code() = 0;
  virtual string cpp_type_name() = 0;
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *>) = 0;
  virtual string equals(const string &prefix, int indent = 0) = 0;
  virtual string test_populate(const string &prefix, int indent = 0) = 0;
  virtual bool is_constant() {return false; }
  virtual string submsg_type() {return "";}
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
    type_spec_str = package + string("/") + spec_name;
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
    type_spec->class_name = nonprimitive_type(package, spec_name);
  }
  virtual string submsg_type() {return cpp_type_name(); }

  virtual string cpp_decl()
  {
    return string("  public ") + cpp_type_name() + string(" ") + name + string(";\n");
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
    return name + string(".serializationLength()");
  }
  virtual bool is_fixed_length()
  {
    return type_spec->is_fixed_length();
  }
  virtual vector<string> cpp_ctor_clauses() {
	  vector<string> v;
	  v.push_back(string("    ") + name + string(" = new ") + type_spec->class_name + string("();\n"));
	  return v;
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> v;
    v.push_back(string("    ") + name + string(" = (") +  type_spec->class_name  + string(")") + name + string(".clone();\n"));
    return v;
  }
  virtual string cpp_copy_ctor_code() {
     return string("    ") + name + string(" = copy.") + name + ";\n"; // string(" = new ") + type_spec->class_name + string("(copy);\n");
  }

  virtual string cpp_assign_code()
  {
    return string("    ") + name + string(" = copy.") + name + string(";\n");
  }
  virtual string cpp_dtor_code() { return string(); }
  virtual string helper_funcs() { return string(); }
  virtual string serialization_code()
  {
//    std::string _name = name;
//    if (_name == "header")
//    {
//      _name = "_ser_header";
//    }

    return string("     ") + name + string(".serialize(bb, seq);\n");
  }
  virtual string deserialization_code()
  {
    return string("     ") + name + string(".deserialize(bb);\n");
  }

  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v)
  {
    vector<msg_spec *> types = type_spec->cpp_types(v);
    bool found = false;
    for (vector<msg_spec *>::iterator i = types.begin();
         !found && i != types.end(); ++i)
      if ((*i)->spec_file == type_spec->spec_file)
        found = true;
    if (!found)
      types.push_back(type_spec);
    return types;
  }
};

class var_array : public msg_var
{
public:
  string eletype;
  string eletype_box;
  string eletype_java;
  int len;
  msg_var *ele_var;
  var_array(const string &_type, const string &_name, msg_spec *_parent)
  : msg_var(_name, _parent), len(0)
  {
    string::size_type open_bracket  = _type.find("[");
    string::size_type close_bracket = _type.find("]");
    eletype = _type.substr(0, open_bracket);
    if (close_bracket > open_bracket + 1)
    {
      string len_str = _type.substr(open_bracket+1,
                                    close_bracket - open_bracket - 1);
      len = atoi(len_str.c_str());
    }
    ele_var = parent->make_var(eletype, "dummy");
    if (msg_spec::is_primitive(eletype)) eletype_box = primitive_type_box(eletype);
    else eletype_box = eletype;

	eletype_java = ele_var->cpp_type_name();
//	if (is_number(eletype)) eletype_java = eletype_box;
  }
  virtual string cpp_decl()
  {
    ostringstream code;
    code << "  public " << cpp_type_name() << " " << name << ";\n";
    //java.util.List<" << (msg_spec::is_primitive(eletype) ? eletype_box : ele_var->cpp_type_name()) << "> "
    return code.str();
  }
  virtual string test_populate(const string &prefix, int indent = 0)
  {
    ostringstream code, iter_name_stream;
    string indent_str;
    for (int i = 0; i < indent; i++)
      indent_str += " ";
    iter_name_stream << name << indent << "_idx";
    string iter_name = iter_name_stream.str();
    ele_var->name = name + string(".get(") + iter_name + string(")");
    if (len)
    {
      code << indent_str << "  for (int " << iter_name << " = 0; "
           << iter_name << " < " << len << "; "
           << iter_name << "++)\n";
      code << indent_str << "  {\n";
      code << indent_str << ele_var->test_populate(prefix, indent + 2);
      code << indent_str << "  }\n";
    }
    else
    {
      code << indent_str << "  if (rand() % 2 != 0)\n  {\n";
      code << indent_str << "    " << prefix << ".set_" << name << "_size(rand() % 10000);\n";
      code << indent_str << "    for (int " << iter_name << " = 0; "
           << iter_name << " < " << prefix << "." << name << ".size(); "
           << iter_name << "++)\n";
      code << indent_str << "    {\n";
      code << indent_str << ele_var->test_populate(prefix, indent + 4);
      code << indent_str << "    }\n  }\n";
    }
    return code.str();
  }
  virtual string equals(const string &prefix, int indent = 0)
  {
/*    ostringstream code, iter_name_stream;
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
           << iter_name  << " < a."
           << (prefix.length() ? prefix + string(".") : "") << name
           << ".size(); " << iter_name << "++)\n";
    ele_var->name = name + string("[") + iter_name + string("]");
    code << indent_str << "  {\n";
    code << indent_str << ele_var->equals(prefix, indent+2);
    code << indent_str << "  }\n";*/

    return string("");
  }

  virtual string submsg_type() {
	  if (!msg_spec::is_primitive(eletype)) return eletype_java;
	  return "";
  }

  virtual string cpp_type_name()
  {
	    return eletype_java + string("[]");
  }
  virtual string cpp_new_type_name(string sz)
 {
	    return eletype_java + string("[") + sz + string("]");
  }
  virtual vector<string> cpp_ctor_clauses()
  {
	  vector<string> ret;

	  ostringstream code;
	  if (len) {
		     ostringstream len_str;
		     len_str << len;
			 code << "  " << name << " = new " << cpp_new_type_name(len_str.str()) << ";" << endl;
			 if (!is_number(eletype)) {
			  	code << "  for(int i = 0; i < " << len << "; i++)";
			    code << "    " << name << "[i] = new " << eletype_java << "();\n";
			 }
	  } else {
			 code << "  " << name << " = new " << cpp_new_type_name(string("0")) << ";" << endl;
	  }

	  ret.push_back(code.str());

	  return ret;
  }
  virtual string cpp_ctor_code()
  {
	  return "";
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
	vector<string> v;
    v.push_back(string("    ") + name + string(" =  (") + cpp_type_name() + string(")(clone.") + name + string(".clone());\n"));
    if (!is_number(eletype) && eletype != "string") {
    	v.push_back(string("    for (int i = 0; i < ") + name + string(".length; i++) ") + name + string("[i] = (")
    			     + eletype_java + string(")")+ name + string("[i].clone();\n"));
    }
    return v;
 }
  virtual string cpp_copy_ctor_code()
  {
    ostringstream code;
//    code << "    " << name << " = new " << cpp_new_type_name() << "(copy." << name << ");\n";
 //   code << "    for(int i = 0; i < copy." << name << ".size(); i++) " << name << ".add(new "
    return code.str();
  }
  virtual string cpp_assign_code()
  {
    return cpp_copy_ctor_code();
  }
  virtual string cpp_dtor_code()
  {
    return string("");
  }
  virtual string length_expr()
  {
    uint32_t CODE_LEN = 4096;
    char code[CODE_LEN];
    if (ele_var->is_fixed_length())
    {
    	ele_var->name = name + string("[0]");

    	if (len) {
    		if (len > 0) {
      	      snprintf(code, CODE_LEN, "%d * (%s)", len, ele_var->length_expr().c_str());
    		} else {
    	      snprintf(code, CODE_LEN, "0");
    		}
    	} else {
    	      snprintf(code, CODE_LEN, "4 + (%s.length == 0 ? 0 : %s.length * (%s))", name.c_str(), name.c_str(), ele_var->length_expr().c_str());
    	}
    } else {
    	snprintf(code, CODE_LEN, "%s + calc_%s_array_serialization_len()",
    			(len ? "0" : "4"), name.c_str());
    }
  		return string(code);
  }
  virtual bool is_fixed_length()
  {
    return ((len && ele_var->is_fixed_length()) ? true : false);
  }
  virtual string helper_funcs()
  {
    ostringstream code;

    if (!ele_var->is_fixed_length()) {
    	ele_var->name = name + string("[i]");
        code << "  int calc_" << name << "_array_serialization_len() {\n"
             << "    int l = 0;\n";
        code << "    for (int i = 0; i < " << name << ".length; i++) \n";
        code << "      l += " << ele_var->length_expr() << ";\n"
             << "    return l;\n  }\n";
    }
/*    if (!len)
    {
      code << "  void set_" << name << "_size(uint32_t __ros_new_size)\n"
           << "  {\n    this->" << name << ".resize(__ros_new_size);\n  }\n"
           << "  inline uint32_t get_" << name << "_size() const { return "
           << name << ".size(); }\n";
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
    }
    // stl vector setters/getters
    code << "  inline void get_" << name << "_vec (std::vector<"
         << ele_var->cpp_type_name() << "> &__ros_vec) const\n"
         << "  {\n    __ros_vec = this->" << name << ";\n  }\n";
    code << "  inline void set_" << name << "_vec(const std::vector<"
         << ele_var->cpp_type_name() << "> &__ros_vec)\n"
         << "  {\n    this->" << name << " = __ros_vec;\n  }\n"; */
    return code.str();
  }
  virtual string serialization_code()
  {
    ostringstream code;
    if (!len) {
      code << "    bb.putInt(" << name << ".length);\n";
    }

    if(is_number(eletype)) {
    	string type_cap = primitive_type_cap(eletype);
		if (type_cap == "Byte") {
	    	code << "    bb.put(" << name << ");\n";
		} else {
	    	code << "    bb.position(bb.position() + bb.as" << type_cap << "Buffer().put(" << name << ").position() * " << number_bytes(eletype) << ");\n";
		}
    } else {
        code << "    for (" << eletype_java << " x : " << name << ")\n";

        if(msg_spec::is_primitive(eletype))
        	code << "      Serialization.write" << primitive_type_cap(eletype) << "(bb, x);\n";
        else
        	code << "      x.serialize(bb, seq);\n";
    }

    return code.str();
  }

  virtual string deserialization_code()
  {
    ostringstream code;
//    string type = primitive_type(eletype);

    if (!len)
        code << "     int " << name << "_len = bb.getInt();\n";
    else
		code << "     int " << name << "_len = " << len << ";\n";

    code << "    " << name << " = new " << cpp_new_type_name(name + string("_len")) << ";\n";

    if(is_number(eletype)) {
    	string type_cap = primitive_type_cap(eletype);
		if (type_cap == "Byte") {
	    	code << "    bb.get(" << name << ");\n";
		} else {
	    	code << "    bb.position(bb.position() + bb.as" << type_cap << "Buffer().get(" << name << ").position() * " << number_bytes(eletype) << ");\n";
		}
    } else {
        code << "    for(int i = 0; i < " << name << "_len; i++)\n";

        if(msg_spec::is_primitive(eletype))
        	code << "      " << name << "[i] = Serialization.read" << primitive_type_cap(eletype) << "(bb);\n";
        else
        	code << "      {" << name << "[i] = new " << eletype_java << "(); " << name << "[i].deserialize(bb); }\n";
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
    return string("  public ") + cpp_type_name() + string(" ") + name + string(";\n");
  }
  virtual string cpp_type_name()
  {
	  return primitive_type(type);
  }
  virtual string length_expr()
  {
    if (type == "byte" || type == "char" || type == "uint8" || type == "int8")
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
      return string("4 + ") + name + string(".length()");
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
      vector<string> c;
      if (type == "string" || type == "time" || type == "duration") {
    	  c.push_back(string("    ") + name + string(" = new ") + cpp_type_name() + string("();\n"));
      }
      return c;
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> v;
    if (!is_number(type) && type != "string") {
    	v.push_back(name + string(" = (") + primitive_type_box(type) + string(")") + name + string(".clone();\n"));
    }

//    if (type == "time")
 //   	v.push_back(name + string(" = new Time(copy.") + name + string(");\n"));
 //   else if (type == "duration")
 //   	v.push_back(name + string(" = new Duration(copy.") + name + string(");\n"));
 //   else
//    	v.push_back();
    return v;
  }
  virtual string cpp_copy_ctor_code() { return name + string(" = copy.") + name + string(";\n"); }
  virtual string cpp_assign_code()
  {
//    return string("    ") + name + string(" = copy.") + name + string(";\n");
	  return string("");
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
        type == "float32" || type == "float64")
      return indent_str + prefix + string(".") + name + string(" = (") +
             primitive_type(type) + string(") Math.random();\n");
    else if (type == "time")
      return indent_str + prefix + "." + name + string(" = new Time((int)Math.random(), (int)Math.random());\n");
    else if (type == "duration")
      return indent_str + prefix + "." + name + string(" = new Duration((int)Math.random(), (int)Math.random());\n");
    else if (type == "string")
      return indent_str + string("if (Math.random() > 0.5)\n") +
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
	  if (is_number(type)) {
		  string ptc = primitive_type_cap(type);
		  if (ptc == "Byte") ptc = "";
		  return string("    bb.put") + ptc + string("(") + name + string(");\n");
	  } else {
		  return string("    Serialization.write") + primitive_type_cap(type) + string("(bb, ") + name + string(");\n");
	  }
  }
  virtual string deserialization_code()
  {
	  if (is_number(type)) {
		  string ptc = primitive_type_cap(type);
		  if (ptc == "Byte") ptc = "";
		  return string("    ") + name + string(" = bb.get") + ptc + string("();\n");
	  } else {
		  return string("    ") + name + string(" = Serialization.read") + primitive_type_cap(type) + string("(bb);\n");
	  }
  }
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v)
  {
    return v;
  }
};

static string gsub(const string &input , const string &fr,
                   const string &to)
{
  if (input.empty())
    return input;
  string s(input);
  string::size_type to_len = to.length();
  string::size_type fr_len = fr.length();
  string::size_type loc = 0;
  while (string::npos != (loc = s.find(fr, loc)))
  {
    s.replace(loc, fr_len, to);
    loc += to_len;
    if (loc >= s.length())
      break;
  }
  return s;
}

class var_constant : public msg_var
{
public:
  string type, constant, safe_constant;
  var_constant(const string &_type, const string &_name,
               const string &_constant, msg_spec *_parent)
  : msg_var(_name, _parent), type(_type), constant(_constant),
    safe_constant(_constant)
  {
    safe_constant = gsub(safe_constant, "\\", "\\\\");
    safe_constant = gsub(safe_constant, "\"", "\\\"");
  }
  virtual ~var_constant() { }
  virtual bool is_constant() {return true; }

  virtual string cpp_decl()
  {
    // for integer types, we can initialize a static version. Otherwise,
    // declare a const member of the class and initialize it in the
    // constructor. That's annoying, because each member of the class will
    // have to drag that instance around, but oh well. If this turns
    // out to be a problem, I could spit out a .cpp file and compile a
    // library. But it's so much easier to keep the whole thing in a header.
      return string("  public static final ") + cpp_type_name() + string(" ") +
             name + string(" = ") + constant + ";\n";
  }
/*
  virtual string cpp_impl_decl()
  {
    if (type != "string")
    {
      // simple case; just dump the constant into the code
      return string("const static ") + cpp_type_name() + string(" ") +
             parent->package + string("::") + parent->spec_name +
             string("::") + name + string(" = ") + constant + string(";\n");
    }
    else
    {
      // a bit more complex. escape the quotes and backslashes, wrap in a ctor
      ostringstream code;
      string safe_constant(constant);
      safe_constant = gsub(safe_constant, "\\", "\\\\");
      safe_constant = gsub(safe_constant, "\"", "\\\"");
      code << "const std::string " << parent->package << "::"
           << parent->spec_name << "::" << name << " = std::string(\""
           << safe_constant << "\");\n";
      return code.str();
    }
  }
*/
  virtual string cpp_type_name()
  {
    // this version of the function purposefully does not
    // include Time or Duration, instead emits a preprocessor error directive
    if (type == "time" || type == "duration")
      return string("\n#error woah! unhandled primitive type ") + type +
             string("\n");
    return primitive_type(type);
  }
  virtual string length_expr() { return "0"; }
  virtual bool is_fixed_length() { return true; }
  virtual vector<string> cpp_ctor_clauses()
  {
    vector<string> code;
    return code;
  }
  virtual vector<string> cpp_copy_ctor_initializers()
  {
    vector<string> code;
    return code;
  }
  virtual string cpp_copy_ctor_code()
  { return string(); }
  virtual string cpp_assign_code() { return string(); }
  virtual string cpp_dtor_code() { return string(); }
  virtual string test_populate(const string &prefix, int indent = 0)
  { return string(); }
  virtual string equals(const string &prefix, int indent = 0)
  { return string(); }
  virtual string helper_funcs() { return string(); }
  virtual string serialization_code() { return string(); }
  virtual string deserialization_code() { return string(); }
  virtual vector<msg_spec *> cpp_types(vector<msg_spec *> v) { return v; }
};

msg_spec::msg_spec(const string &_spec_file, const string &_package,
                   const string &_spec_name, const string &_pkg_path,
                   bool is_filename, bool _is_root)
: spec_file(_spec_file), spec_name(_spec_name), package(_package),
  pkg_path(_pkg_path), is_root(_is_root)
{
  std::string memory_file;

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
          throw std::runtime_error("couldn't launch gendeps in genmsg_cpp\n");
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

    fseek(f, 0, SEEK_END);
    int len = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* buffer = new char[len];
    fread(buffer, len, 1, f);
    memory_file = std::string(buffer, len);
    delete buffer;

    fclose(f);
  }
  else
  {
    memory_file = spec_file;
  }

  vector<string> lines;
  string_split(memory_file, lines, "\n");
  for (size_t i = 0; i < lines.size(); i++)
  {
    lines[i] += "\n";
    char linebuf[1024];
    strncpy(linebuf, lines[i].c_str(), sizeof(linebuf)-1);
    process_line(linebuf, i+1);
    //printf("line = [%s]\n", lines[i].c_str());
  }
}

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
  vars.push_back(new var_constant(type, name, constant, this));
  return true;
}

bool msg_spec::is_integer(const std::string &type)
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

bool msg_spec::is_primitive(const string &type)
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

void msg_spec::emit_cpp_includes(FILE *f)
{
  assert(f);
  vector<msg_spec *> sub_specs;
  sub_specs = cpp_types(sub_specs);
//  fprintf(f, "import ros.communication.Message;\n");
//  fprintf(f, "import ros.communication.Time;\n");
//  fprintf(f, "import ros.communication.Duration;\n");
  fprintf(f, "import java.nio.ByteBuffer;\n\n");
   /*if (sub_specs.size() > 0)
  {
    for (vector<msg_spec *>::iterator i = sub_specs.begin();
         i != sub_specs.end(); ++i)
        fprintf(f, "import ros.pkg.%s.msg.%s;\n", (*i)->package.c_str(),
                (*i)->spec_name.c_str());
    fprintf(f, "\n");
  }*/
}

void msg_spec::emit_cpp_class(FILE *f, bool for_srv, const string &service_name)
{
  assert(f);
  if (!for_srv)
  {
	    fprintf(f, "package ros.pkg.%s.msg;\n\n\n", package.c_str());

    emit_cpp_includes(f);
  }

//  fprintf(f, "//! \\htmlinclude %s.msg.html\n\n",g_name.c_str());
  fprintf(f, "public %s class %s extends ros.communication.Message\n{\n\n", (for_srv ? "static" : ""), g_name.c_str());

  bool header_present = false;
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    fprintf(f, "%s", (*v)->cpp_decl().c_str());
    if ((*v)->name == "header" &&
        ((*v)->cpp_type_name().find("Header") != string::npos))
      header_present = true;
  }
  fprintf(f, "\n  public %s() {\n super();\n", g_name.c_str());
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    vector<string> strs = (*v)->cpp_ctor_clauses();
    for (vector<string>::iterator i = strs.begin(); i != strs.end(); ++i)
      fprintf(f, "%s", (*i).c_str());
  }
  fprintf(f, "\n  }\n");
//  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
//    fprintf(f, "%s", (*v)->cpp_ctor_code().c_str());
//  fprintf(f, "  }\n");

  /*  fprintf(f, "  public %s(%s copy) {\n",
          g_name.c_str(), g_name.c_str());
   for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    fprintf(f, "%s", (*v)->cpp_copy_ctor_code().c_str());
  fprintf(f, "  }\n");*/

//  fprintf(f, "  %s &operator =(const %s &copy)\n  {\n",
//          g_name.c_str(), g_name.c_str());
//  fprintf(f, "    if (this == &copy)\n      return *this;\n");
//  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
//    fprintf(f, "%s", (*v)->cpp_dtor_code().c_str());
//  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
//    fprintf(f, "%s", (*v)->cpp_assign_code().c_str());
//  fprintf(f, "    return *this;\n  }\n");
//  fprintf(f, "  virtual ~%s() \n  {\n", g_name.c_str());
//  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
//    fprintf(f, "%s", (*v)->cpp_dtor_code().c_str());
//  fprintf(f, "  }\n");
  fprintf(f, "  public static java.lang.String __s_getDataType() { return \"%s/%s\"; }\n",
          g_pkg.c_str(), (!for_srv ? g_name.c_str() : (service_name + g_name).c_str()));
  fprintf(f, "  public static java.lang.String __s_getMD5Sum() { return \"%s\"; }\n",
          md5sum.c_str());

  fprintf(f, "  public static java.lang.String __s_getMessageDefinition()\n  {\n");
  fprintf(f, "    return \n");

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

    fprintf(f, "    \"%s\\n\" + \n", line.c_str());
  }
  fprintf(f, "    \"\";\n");
  fprintf(f, "  }\n");


  fprintf(f, "  public java.lang.String getDataType() { return __s_getDataType(); }\n");
  fprintf(f, "  public java.lang.String getMD5Sum()   { return __s_getMD5Sum(); }\n");
  fprintf(f, "  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }\n");
  if (server_md5sum.length())
  {
    fprintf(f, "  public static java.lang.String __s_getServerMD5Sum() { return (\"%s\"); }\n", server_md5sum.c_str());
    fprintf(f, "  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }\n");

    fprintf(f, "  public static java.lang.String  __s_getServiceDataType() { return (\"%s\"); }\n", service_datatype.c_str());
    fprintf(f, "  public java.lang.String getServiceDataType() { return __s_getServiceDataType(); }\n");
  }

  fprintf(f, "  public %s clone() {\n    %s clone = (%s)super.clone();\n", g_name.c_str(),g_name.c_str(), g_name.c_str());
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v) {
	  vector<string> inits = (*v)->cpp_copy_ctor_initializers();
	  for (vector<string>::iterator it = inits.begin(); it != inits.end(); ++it) {
		  fprintf(f, "  %s", it->c_str());
	  }
  }
  fprintf(f, "    return clone;\n  }\n\n");

  fprintf(f, "  public static java.util.Map<java.lang.String, java.lang.String> fieldTypes() {\n    ");
  fprintf(f, "     java.util.HashMap<java.lang.String, java.lang.String> m = new java.util.HashMap<java.lang.String, java.lang.String>  (); ");

  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v) {
	  if (!(*v)->is_constant()) {
		  fprintf(f, "     m.put(\"%s\", \"%s\");\n", (*v)->name.c_str(), (*v)->cpp_type_name().c_str());
	  }
  }
  fprintf(f, "     return m;\n  }\n\n");

  fprintf(f, "  public static java.util.Set<java.lang.String> submessageTypes() {\n    ");
  fprintf(f, "     java.util.HashSet<java.lang.String> s = new java.util.HashSet<java.lang.String>  (); ");

  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v) {
	  string sm = (*v)->submsg_type();
	  if (sm.size() > 0) {
		  fprintf(f, "     s.add(\"%s\");\n", sm.c_str());
	  }
  }
  fprintf(f, "     return s;\n  }\n\n");

  fprintf(f, "  public void setTo(ros.communication.Message __m) {\n");
  fprintf(f, "    if (!(__m instanceof %s)) throw new RuntimeException(\"Invalid Type\");\n", g_name.c_str());
  fprintf(f, "    %s __m2 = (%s) __m;\n", g_name.c_str(), g_name.c_str());
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v) {
	  if (!(*v)->is_constant()) {
		  fprintf(f, "    %s = __m2.%s;\n", (*v)->name.c_str(), (*v)->name.c_str());
	  }
  }
  fprintf(f, "    }\n\n");


  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    string h = (*v)->helper_funcs();
    if (h.length() > 0)
      fputs(h.c_str(), f);
  }
  fputs(serializationLength_func().c_str(), f);
  fputs("  public void serialize(ByteBuffer bb, int seq) {\n", f);

  if (header_present)
  {
    fputs("    ros.pkg.roslib.msg.Header _ser_header = header;\n", f);
    fputs("    boolean __reset_seq = (header.seq == 0);\n" \
          "    if (__reset_seq) _ser_header.seq = seq;\n" \
          "    boolean __reset_timestamp = header.stamp.isZero();\n" \
          "    if (__reset_timestamp)\n" \
          "      _ser_header.stamp = ros.Ros.getInstance().now();\n", f);
  }

  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    fputs((*v)->serialization_code().c_str(), f);

  fputs("  }\n", f);

  fputs("  public void deserialize(ByteBuffer bb)  {\n", f);
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
    fputs((*v)->deserialization_code().c_str(), f);
  fputs("  }\n", f);
  fputs("}\n\n", f);
//  fprintf(f, "typedef boost::shared_ptr<%s> %sPtr;\n", g_name.c_str(), g_name.c_str());
//  fprintf(f, "typedef boost::shared_ptr<%s const> %sConstPtr;\n\n", g_name.c_str(), g_name.c_str());
 // if (!for_srv)
//	  fputs("\n}\n\n", f);
}

string msg_spec::serializationLength_func()
{
  string s;
//  if (!is_fixed_length())
    s += "  public int serializationLength() \n  {\n";
//  else
//    s += "  static uint32_t s_serializationLength()\n  {\n";
  s += "    int __l = 0;\n";
  for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
  {
    // Is it an array of non-primitives?
    // If so, then we need declare the dummy variable that
    // will be used to determine serialization length
    /*
    var_array* array_ptr = dynamic_cast<var_array*>(*v);
    if(array_ptr && !msg_spec::is_primitive(array_ptr->eletype))
      s += string("    {\n    ") + array_ptr->ele_var->cpp_decl() +
              string("  ");
    */

    s += string("    __l += ") + (*v)->length_expr() + string("; // ") +
      (*v)->name + string("\n");
    /*
    if(array_ptr && !msg_spec::is_primitive(array_ptr->eletype))
      s += string("    }\n");
    */
  }
  s += "    return __l;\n";
  s += "  }\n";
  return s;
}

string msg_spec::to_list_func() { return string(""); }

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

