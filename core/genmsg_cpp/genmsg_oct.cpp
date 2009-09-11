/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

// author: Rosen Diankov
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <cstdio>
#include <cerrno>
#include <cassert>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/param.h>
#include <cstdlib>
#include <cstring>
#include "msgspec.h"
#include "utils.h"

using namespace std;

bool g_header_done;

class msg_var
{
public:
    string name;
    msg_spec *parent;
    msg_var(const string &_name, msg_spec *_parent)
        : name(_name), parent(_parent) { }
    virtual ~msg_var() { }
    virtual string oct_decl() = 0;
    virtual string length_expr(const string& msgname) = 0;
    virtual bool is_fixed_length() = 0;
    virtual string oct_ctor(const string& msgname) = 0;
    virtual string helper_funcs() { return string(); }
    virtual string serialization_code(const string& msgname, int& nExpectedCount) = 0;
    virtual string deserialization_code(const string& msgname) = 0;
    virtual string oct_type_name() = 0;
    virtual vector<msg_spec *> oct_types(vector<msg_spec *>) = 0;
    virtual bool is_primitive() { return false; }
    virtual string get_fullname(const string& msgname) {
        if( name.size() > 0 )
            return msgname + string(".") + name;
        return msgname;
    }
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
        if (type_vec.size() >= 2) {
            package = type_vec[0];
            spec_name = type_vec[1];
        }
        else
            spec_name = type_vec[0];
        string spec_file = spec_name + string(".msg");
        if (spec_name == "Header")
            package = "roslib";
        if(_parent->is_root && !rospack_check_dep(g_pkg, package)) {
                printf("Error: you're using package %s in a message/service, but you don't "
                       "declare a dependency on that package.  Please add the missing "
                       "dependency to your manifest.xml.\n", package.c_str());
                exit(13);
        }
        pkg_path = rospack_find(package);
        if (pkg_path.length() == 0) {
            printf("Error: couldn't find package %s which was referenced in the "
                   "message.\n", package.c_str());
            exit(13);
        }
        spec_file = pkg_path + string("/msg/") + spec_file;
        //printf("spec file = [%s]\n", spec_file.c_str());
        type_spec_str = package + string("/") + spec_name;
        try {
            type_spec = new msg_spec(spec_file, package, spec_name, pkg_path);
        }
        catch(std::runtime_error err) {
            printf("Error: \"%s\" is neither a primitive type, nor a derived "
                   "type that I can find.\n", spec_name.c_str());
            exit(13);
        }
        type_spec->class_name = package + string("_") + spec_name;
    }
    virtual string oct_decl()
    {
        return string("  ") + oct_type_name() + string(" ") + name;
    }
    virtual string oct_type_name() { return type_spec->class_name; }
    virtual string length_expr(const string& msgname)
    {
        stringstream ss;
        if( is_fixed_length() )
            if( name.size() > 0 )
                ss << get_fullname(msgname) << string(".serializationLength_(") << get_fullname(msgname) << string(")"); 
            else // create a dummy class that can be queried for the length
                ss << type_spec->class_name << string("().serializationLength_()");
        else
            ss << get_fullname(msgname) + string(".serializationLength_(") << get_fullname(msgname) << ")";
        
        return ss.str();
    }
    virtual bool is_fixed_length()
    {
        return type_spec->is_fixed_length();
    }
    virtual string oct_ctor(const string& msgname)
    {
        return get_fullname(msgname) + string(" = ") + oct_type_name() + string("();\n");
    }
    virtual string serialization_code(const string& msgname, int& nExpectedCount)
    {
        stringstream ss;
        ss << get_fullname(msgname) << ".serialize_(" << get_fullname(msgname) << ", seq__, fid__);" << endl;
        return ss.str();
    }
    virtual string deserialization_code(const string& msgname)
    {
        stringstream ss;
        ss << get_fullname(msgname) << " = " << type_spec->class_name << "();" << endl;
        ss << get_fullname(msgname) << " = " << get_fullname(msgname) << ".deserialize_(" << get_fullname(msgname) << ", fid__);" << endl;
        return ss.str();
    }
    virtual vector<msg_spec *> oct_types(vector<msg_spec *> v)
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
                len = atoi(len_str.c_str()); // todo: verify this string is all digits
            }
        ele_var = parent->make_var(eletype, "");
    }
    virtual string oct_decl()
    {
        return ele_var->oct_type_name() + string("{} ") + name;
    }
    virtual string oct_type_name()
    {
        return (ele_var->is_primitive() && ele_var->is_fixed_length()) ? "array" : "cell";
    }
    virtual string oct_ctor(const string& msgname)
    {
        stringstream ss;
        if( ele_var->is_primitive() && ele_var->is_fixed_length() ) {
            if( len > 0 )
                ss << get_fullname(msgname) << " = zeros(" << len << ", 1, '" << ele_var->oct_type_name() << "');" << endl;
            else
                ss << get_fullname(msgname) << " = [];" << endl;
        }
        else {
            if( len > 0 ) {
                ss << get_fullname(msgname) << " = cell(" << len << ", 1);" << endl;
                // intialize the values
                ss << "for " << name << "_i__ = 1:" << len << endl
                   << "    " << ele_var->oct_ctor(get_fullname(msgname) + string("{") + name + string("_i__}"))
                   << "end" << endl;
            }
            else
                ss << get_fullname(msgname) << " = {};" << endl;
        }

        return ss.str();
    }
    virtual string length_expr(const string& msgname)
    {
        stringstream ss;
        if( ele_var->is_fixed_length() ) {
            if( len > 0 )
                ss << len << " * (" << ele_var->length_expr("") << ")";
            else
                ss << "4 + numel(" << get_fullname(msgname) << ") * (" << ele_var->length_expr("") << ")";
        }
        else {
            if( len == 0 )
                ss << "4 + ";
            ss << g_pkg << "_" << g_name << "___sum_array_length__(cellfun(@(" << name << "_elt__) " << ele_var->length_expr(name + string("_elt__")) << ", " << get_fullname(msgname);
            if( len > 0 )
                ss << "(1:" << len << ")";
            ss << "))";
        }

        return ss.str();
    }
    virtual bool is_fixed_length()
    {
        return len && ele_var->is_fixed_length();
    }
    virtual string serialization_code(const string& msgname, int& nExpectedCount)
    {
        stringstream ss;
        if( len == 0 )
            ss << "fwrite(fid__, numel(" << get_fullname(msgname) << "), 'uint32');" << endl;
        if( ele_var->is_primitive() && ele_var->is_fixed_length() ) {
            if( len > 0 ) {
                nExpectedCount += len;
                ss << "c__ = c__ + ";
            }
            ss << "fwrite(fid__, " << get_fullname(msgname);
            if( len > 0 )
                ss << "(1:" << len << ")";
            else
                ss << "(:)";
            ss << ", '" << ele_var->oct_type_name() << "');" << endl;
        }
        else {
            ss << "for " << name << "_i__ = 1:numel(" << get_fullname(msgname) << ")" << endl << "    ";
            ss << ele_var->serialization_code(get_fullname(msgname) + string("{") + name + string("_i__}"), nExpectedCount);
            ss << "end" << endl;
        }
        return ss.str();
    }
    virtual string deserialization_code(const string& msgname)
    {
        stringstream ss;
        if( len == 0 )
            ss << "size__ = double(fread(fid__, 1, 'uint32=>uint32'));" << endl;

        if( ele_var->is_primitive() && ele_var->is_fixed_length() ) {
            ss << get_fullname(msgname);
            if( len == 0 )
                ss << " = fread(fid__, size__";
            else
                ss << "(1:" << len << ") = fread(fid__, " << len;
            ss << ", '" << ele_var->oct_type_name();
            if( ele_var->oct_type_name() == "single" ) // for some reason octave 3.0.0 doesn't support single=>single
                ss << "');" << endl;
            else
                ss << "=>" << ele_var->oct_type_name() << "');" << endl;
        }
        else {
            ss << get_fullname(msgname) << " = cell(";
            if( len == 0 )
                ss << "size__";
            else
                ss << len;
            ss << ", 1);" << endl;
            ss << "for " << name << "_i__ = 1:";
            if( len == 0 )
                ss << "size__";
            else
                ss << len;
            ss << endl << "    " << ele_var->deserialization_code(get_fullname(msgname) + string("{") + name + string("_i__}"));
            ss << "end\n";
        }

        return ss.str();
    }
    virtual vector<msg_spec *> oct_types(vector<msg_spec *> v)
    {
        return ele_var->oct_types(v);
    }
};

class var_primitive : public msg_var
{
public:
    string type;
    var_primitive(const string &_type, const string &_name, msg_spec *_parent)
        : msg_var(_name, _parent), type(_type) { }
    virtual ~var_primitive() { }
    virtual string oct_decl()
    {
        return oct_type_name() + string(" ") + name;
    }
    virtual string oct_type_name()
    {
        if (type == "byte" || type == "int8")
            return "int8";
        else if (type == "char" || type == "uint8")
            return "uint8";
        else if (type == "uint16")
            return "uint16";
        else if (type == "int16")
            return "int16";
        else if (type == "uint32")
            return "uint32";
        else if (type == "int32")
            return "int32";
        else if (type == "uint64")
            return "uint64";
        else if (type == "int64")
            return "int64";
        else if (type == "float32")
            return "single";
        else if (type == "float64")
            return "double";
        else if (type == "string")
            return "string";
        else if (type == "time")
            return "rosoct_time";
        else if (type == "duration")
            return "rosoct_duration";
        else
            return string("\n#error woah! unhandled primitive type ") + type +
                string("\n");
    }
    virtual string length_expr(const string& msgname)
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
            return string("4 + numel(") + get_fullname(msgname) + string(")");
        else
            return "\n#error woah! bogus length_expr in var_primitive\n";
    }
    virtual bool is_fixed_length()
    {
        return type != "string";
    }
    virtual bool is_primitive()
    {
        return type != "time" && type != "duration";
    }
    virtual string oct_ctor(const string& msgname)
    {
        if (type == "string")
            return get_fullname(msgname) + string(" = '';\n");
        else if (type == "time" || type == "duration") {
            return get_fullname(msgname) + string(" = struct('sec',uint32(0),'nsec',uint32(0));\n");
        }
        else
            return get_fullname(msgname) + string(" = ") + oct_type_name() + string("(0);\n");
    }
    virtual string serialization_code(const string& msgname, int &nExpectedCount)
    {
        stringstream ss;
        if( type == "string") {
            ss << "fwrite(fid__, numel(" << get_fullname(msgname) << "), 'uint32');" << endl;
            ss << "fwrite(fid__, " << get_fullname(msgname) << ", 'uint8');" << endl;
        }
        else if (type == "time" || type == "duration") {
            ss << "c__ = c__ + fwrite(fid__, " << get_fullname(msgname) << ".sec" << ", 'uint32');" << endl;
            ss << "c__ = c__ + fwrite(fid__, " << get_fullname(msgname) << ".nsec" << ", 'uint32');" << endl;
            nExpectedCount += 2;
        }
        else {
            nExpectedCount += 1;
            ss << "c__ = c__ + fwrite(fid__, " << get_fullname(msgname) << ", '" << oct_type_name() << "');" << endl;
        }

        return ss.str();
    }
    virtual string deserialization_code(const string& msgname)
    {
        stringstream ss;
        if( type == "string") {
            ss << "size__ = double(fread(fid__, 1,'uint32=>uint32'));" << endl;
            ss << get_fullname(msgname) << " = fread(fid__, size__, '*char')';" << endl;
        }
        else if (type == "time" || type == "duration") {
            ss << get_fullname(msgname) << ".sec" << " = fread(fid__,1,'uint32=>uint32');" << endl;
            ss << get_fullname(msgname) << ".nsec" << " = fread(fid__,1,'uint32=>uint32');" << endl;
        }
        else {
            ss << get_fullname(msgname) << " = fread(fid__,1,'" << oct_type_name();
            if( oct_type_name() == "single" ) // for some reason octave 3.0.0 doesn't support single=>single
                ss << "');" << endl;
            else
                ss  << "=>" << oct_type_name() << "');" << endl;
        }
        return ss.str();
    }
    virtual vector<msg_spec *> oct_types(vector<msg_spec *> v)
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
    virtual string oct_decl()
    {
        return string("CONST ") + oct_type_name() + string("() ") + name + string(" = ") + constant;
    }
    virtual string oct_ctor(const string& msgname)
    {
        return get_fullname(msgname) + string(" = @") + g_pkg + string("_") + g_name + string("_") + name + string(";\n");
    }

    virtual string oct_type_name()
    {
        // this version of the function purposefully does not
        // include Time or Duration, instead emits a preprocessor error directive
        if (type == "char" || type == "uint8")
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
        else
            return string("\n#error woah! unhandled primitive type ") + type +
                string("\n");
    }
    virtual string length_expr(const string& msgname) { return "0"; }
    virtual bool is_fixed_length() { return true; }
    virtual string helper_funcs()
    {
        stringstream ss;
        ss << "function x = " << g_pkg << "_" << g_name << "_" << name << "()" << endl;
        ss << "x = " << constant << ";" << endl << endl;
        return ss.str();
    }
    virtual string serialization_code(const string& msgname, int &nExpectedCount) { return string(); }
    virtual string deserialization_code(const string& msgname) { return string(); }
    virtual vector<msg_spec *> oct_types(vector<msg_spec *> v) { return v; }
};

msg_spec::msg_spec(const string &_spec_file, const string &_package,
                   const string &_spec_name, const string &_pkg_path,
                   bool is_filename, bool _is_root)
    : spec_file(_spec_file), spec_name(_spec_name), package(_package),
      pkg_path(_pkg_path), is_root(_is_root)
{
    if (is_filename) {
        // parse it
        FILE *f = fopen(spec_file.c_str(), "r");
        if (!f)
            throw std::runtime_error("couldn't open spec file");

        if (is_root) {
            // compute md5sum
            string cmd = string("`rospack find roslib`/scripts/gendeps --md5 ") + spec_file;
            FILE *md5pipe = popen(cmd.c_str(), "r");
            if (!md5pipe)
                throw std::runtime_error("couldn't launch md5sum in genmsg_oct\n");
            char md5buf[PATH_MAX];
            if (!fgets(md5buf, PATH_MAX, md5pipe))
                throw std::runtime_error("couldn't read md5sum pipe in genmsg_oct\n");
            char *md5str = strtok(md5buf, " \t\n");
            md5sum = string(md5str);
            // call pclose sometime
        }

        const int LINEBUF_LEN = 1024;
        char linebuf[LINEBUF_LEN];

        for (int linenum = 1; !feof(f); linenum++) {
            if (!fgets(linebuf, LINEBUF_LEN, f))
                break; // hit EOF
            process_line(linebuf, linenum); // no need for error checking b/c
            // they are all fatal
        }
        fclose(f);
    }
    else {
        vector<string> lines;
        string_split(string(spec_file), lines, "\n");
        for (size_t i = 0; i < lines.size(); i++) {
            lines[i] += "\n";
            char linebuf[1024];
            strncpy(linebuf, lines[i].c_str(), sizeof(linebuf)-1);
            process_line(linebuf, i+1);
            //printf("line = [%s]\n", lines[i].c_str());
        }
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
    if (!token) { // this should never happen due to checks above
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
    if (!token) {
        printf("woah! on line %d of %s, there was only one token.\n",
               linenum, spec_file.c_str());
        printf("each line needs to have two tokens: the first is the type\n" \
               "and the second is the variable name.\n");
        exit(6);
    }
    name = string(token);
    // ignore the equals sign if it's after a hash
    if (equals_pos == string::npos ||
        (hash_pos != string::npos && equals_pos > hash_pos)) {
        // this was a variable declaration. nothing more to parse; get out of here.
        vars.push_back(make_var(type, name));
        return true;
    }
    if (!constant_allowed) {
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
    if (type != "string") {
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
    set<string> setpkgpaths;
    if (sub_specs.size() > 0) {
        for (vector<msg_spec *>::iterator i = sub_specs.begin(); i != sub_specs.end(); ++i) {
            string pkg_path = rospack_find((*i)->package);
            if( pkg_path.size() > 0 )
                setpkgpaths.insert(pkg_path + "/msg/oct/" + (*i)->package);
            else {
                printf("Error: could not find package %s.\n", (*i)->package.c_str());
                exit(25);
            }
        }
    }

    if( setpkgpaths.size() > 0 ) {
        fprintf(f, "persistent pathsadded__\n"
                "if (isempty (pathsadded__))\n"
                "    pathsadded__ = 1;\n");
        
        for(set<string>::iterator it = setpkgpaths.begin(); it != setpkgpaths.end(); ++it)
            fprintf(f, "    addpath('%s');\n", it->c_str());
        fprintf(f, "end\n\n");
    }
}

void msg_spec::emit_cpp_class(FILE *f, bool for_srv, const string &srv_name)
{
    assert(f);

    if(!g_header_done)
    {
      fprintf(f, "%% Auto-generated.  Do not edit!\n\n");
      g_header_done = true;
    }

    fprintf(f, "%% msg = %s_%s()\n%%\n%% %s message type, fields include:\n", g_pkg.c_str(), g_name.c_str(), g_name.c_str());

    bool header_present = false;
    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v) {
        fprintf(f, "%% %s\n", (*v)->oct_decl().c_str());
        if ((*v)->name == "header" &&
            ((*v)->oct_type_name().find("Header") != string::npos))
            header_present = true;
    }

    // skip a line so it isn't generated by help file
    fprintf(f, "\n%% //! \\htmlinclude %s.msg.html\n",g_name.c_str());
    fprintf(f, "function msg = %s_%s()\n", g_pkg.c_str(), g_name.c_str());

    emit_cpp_includes(f);

    fprintf(f, "\nmsg = [];\n");

    if( for_srv && g_name == string("Request") ) 
        fprintf(f, "msg.create_response_ = @%s_Response;\n", g_pkg.c_str());
    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
        fprintf(f, "%s", (*v)->oct_ctor("msg").c_str());

    // add the internal functions
    fprintf(f, "msg.md5sum_ = @%s_%s___md5sum;\n", g_pkg.c_str(), g_name.c_str());
    if (server_md5sum.length())
        fprintf(f, "msg.server_md5sum_ = @%s_%s___server_md5sum;\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "msg.type_ = @%s_%s___type;\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "msg.serializationLength_ = @%s_%s___serializationLength;\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "msg.serialize_ = @%s_%s___serialize;\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "msg.deserialize_ = @%s_%s___deserialize;\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "\n");

    fprintf(f, "function x = %s_%s___md5sum()\nx = '%s';\n\n", g_pkg.c_str(), g_name.c_str(), md5sum.c_str());

    if (server_md5sum.length())
        fprintf(f, "function x = %s_%s___server_md5sum()\nx = '%s';\n\n", g_pkg.c_str(), g_name.c_str(), server_md5sum.c_str());

    fprintf(f, "function x = %s_%s___type()\nx = '%s/%s';\n\n",
            g_pkg.c_str(), g_name.c_str(), g_pkg.c_str(),
            (!for_srv ? g_name.c_str() : (srv_name + g_name).c_str()));

    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
        fprintf(f, "%s", (*v)->helper_funcs().c_str());

    // serialization length
    fprintf(f, "%s", serializationLength_func().c_str());

    // serialize
    fprintf(f, "function dat__ = %s_%s___serialize(msg__, seq__, fid__)\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "global rosoct\n\n");
    fprintf(f, "c__ = 0;\n"
            "file_created__ = 0;\n"
            "if( ~exist('fid__','var') )\n"
            "    fid__ = tmpfile();\n"
            "    file_created__ = 1;\n"
            "end\n");

    if (header_present)
        fputs("if (msg__.header.seq == 0)\n    msg__.header.seq = seq__;\nend\n"
              "if (msg__.header.stamp.sec == 0 && msg__.header.stamp.nsec == 0)\n    msg__.header.stamp = rosoct_time_now();\nend\n", f);
    int nExpectedCount = 0;
    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
        fputs((*v)->serialization_code("msg__", nExpectedCount).c_str(), f);

    if( nExpectedCount > 0 ) {
        fprintf(f,"if( c__ ~= %d )\n"
                "    error('some members of msg %s:%s are initialized incorrectly!');\n"
                "end\n", nExpectedCount, g_pkg.c_str(), g_name.c_str());
    }

    fprintf(f,"if( file_created__ )\n"
            "    fseek(fid__,0,SEEK_SET);\n"
            "    dat__ = fread(fid__,Inf,'uint8=>uint8');\n"
            "    fclose(fid__);\n"
            "end\n\n");

    // deserialize
    fprintf(f, "function msg__ = %s_%s___deserialize(dat__, fid__)\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "msg__ = %s_%s();\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "file_created__ = 0;\n"
            "if( ~exist('fid__','var') )\n"
            "    fid__ = tmpfile();\n"
            "    file_created__ = 1;\n"
            "    fwrite(fid__,dat__,'uint8');\n"
            "    fseek(fid__,0,SEEK_SET);\n"
            "end\n");

    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
        fputs((*v)->deserialization_code("msg__").c_str(), f);

    fprintf(f, "if( file_created__ )\n"
            "    fclose(fid__);\n"
            "end\n");

    // extra
    fprintf(f, "function l__ = %s_%s___sum_array_length__(x)\n"
            "if( ~exist('x','var') || isempty(x) )\n"
            "    l__ = 0;\n"
            "else\n"
            "    l__ = sum(x(:));\n"
            "end\n\n", g_pkg.c_str(), g_name.c_str());

//    if (!for_srv)
//        fputs("\n}\n\n", f);
}

string msg_spec::serializationLength_func()
{
    stringstream ss;
    ss << "function l__ = " << g_pkg << "_" << g_name << "___serializationLength(msg)" << endl;

    if( vars.size() > 0 )
        ss << "l__ = ";
    else
        ss << "l__ = 0";

    for (vector<msg_var *>::iterator v = vars.begin(); v != vars.end(); ++v)
        ss << " ..." << endl << "    + " << (*v)->length_expr("msg");

    ss << ";" << endl << endl;
    return ss.str();
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
        types = (*v)->oct_types(types);
    return types;
}

string msg_spec::equals(const string &prefix, int indent)
{
    return "";
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
