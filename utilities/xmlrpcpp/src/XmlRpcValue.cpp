
#include "XmlRpcValue.h"
#include "XmlRpcException.h"
#include "XmlRpcUtil.h"
#include "base64.h"

#ifndef MAKEDEPEND
# include <iostream>
# include <ostream>
# include <stdlib.h>
# include <stdio.h>
#endif

#include <sstream>

namespace XmlRpc {


  static const char VALUE_TAG[]     = "<value>";
  static const char VALUE_ETAG[]    = "</value>";

  static const char BOOLEAN_TAG[]   = "<boolean>";
  static const char BOOLEAN_ETAG[]  = "</boolean>";
  static const char DOUBLE_TAG[]    = "<double>";
  static const char DOUBLE_ETAG[]   = "</double>";
  static const char INT_TAG[]       = "<int>";
  static const char I4_TAG[]        = "<i4>";
  static const char I4_ETAG[]       = "</i4>";
  static const char STRING_TAG[]    = "<string>";
  static const char DATETIME_TAG[]  = "<dateTime.iso8601>";
  static const char DATETIME_ETAG[] = "</dateTime.iso8601>";
  static const char BASE64_TAG[]    = "<base64>";
  static const char BASE64_ETAG[]   = "</base64>";

  static const char ARRAY_TAG[]     = "<array>";
  static const char DATA_TAG[]      = "<data>";
  static const char DATA_ETAG[]     = "</data>";
  static const char ARRAY_ETAG[]    = "</array>";

  static const char STRUCT_TAG[]    = "<struct>";
  static const char MEMBER_TAG[]    = "<member>";
  static const char NAME_TAG[]      = "<name>";
  static const char NAME_ETAG[]     = "</name>";
  static const char MEMBER_ETAG[]   = "</member>";
  static const char STRUCT_ETAG[]   = "</struct>";


      
  // Format strings
  std::string XmlRpcValue::_doubleFormat("%.16g");



  // Clean up
  void XmlRpcValue::invalidate()
  {
    switch (_type) {
      case TypeString:    delete _value.asString; break;
      case TypeDateTime:  delete _value.asTime;   break;
      case TypeBase64:    delete _value.asBinary; break;
      case TypeArray:     delete _value.asArray;  break;
      case TypeStruct:    delete _value.asStruct; break;
      default: break;
    }
    _type = TypeInvalid;
    _value.asBinary = 0;
  }

  
  // Type checking
  void XmlRpcValue::assertTypeOrInvalid(Type t)
  {
    if (_type == TypeInvalid)
    {
      _type = t;
      switch (_type) {    // Ensure there is a valid value for the type
        case TypeString:   _value.asString = new std::string(); break;
        case TypeDateTime: _value.asTime = new struct tm();     break;
        case TypeBase64:   _value.asBinary = new BinaryData();  break;
        case TypeArray:    _value.asArray = new ValueArray();   break;
        case TypeStruct:   _value.asStruct = new ValueStruct(); break;
        default:           _value.asBinary = 0; break;
      }
    }
    else if (_type != t)
      throw XmlRpcException("type error");
  }

  void XmlRpcValue::assertArray(int size) const
  {
    if (_type != TypeArray)
      throw XmlRpcException("type error: expected an array");
    else if (int(_value.asArray->size()) < size)
      throw XmlRpcException("range error: array index too large");
  }


  void XmlRpcValue::assertArray(int size)
  {
    if (_type == TypeInvalid) {
      _type = TypeArray;
      _value.asArray = new ValueArray(size);
    } else if (_type == TypeArray) {
      if (int(_value.asArray->size()) < size)
        _value.asArray->resize(size);
    } else
      throw XmlRpcException("type error: expected an array");
  }

  void XmlRpcValue::assertStruct()
  {
    if (_type == TypeInvalid) {
      _type = TypeStruct;
      _value.asStruct = new ValueStruct();
    } else if (_type != TypeStruct)
      throw XmlRpcException("type error: expected a struct");
  }


  // Operators
  XmlRpcValue& XmlRpcValue::operator=(XmlRpcValue const& rhs)
  {
    if (this != &rhs)
    {
      invalidate();
      _type = rhs._type;
      switch (_type) {
        case TypeBoolean:  _value.asBool = rhs._value.asBool; break;
        case TypeInt:      _value.asInt = rhs._value.asInt; break;
        case TypeDouble:   _value.asDouble = rhs._value.asDouble; break;
        case TypeDateTime: _value.asTime = new struct tm(*rhs._value.asTime); break;
        case TypeString:   _value.asString = new std::string(*rhs._value.asString); break;
        case TypeBase64:   _value.asBinary = new BinaryData(*rhs._value.asBinary); break;
        case TypeArray:    _value.asArray = new ValueArray(*rhs._value.asArray); break;
        case TypeStruct:   _value.asStruct = new ValueStruct(*rhs._value.asStruct); break;
        default:           _value.asBinary = 0; break;
      }
    }
    return *this;
  }


  // Predicate for tm equality
  static bool tmEq(struct tm const& t1, struct tm const& t2) {
    return t1.tm_sec == t2.tm_sec && t1.tm_min == t2.tm_min &&
            t1.tm_hour == t2.tm_hour && t1.tm_mday == t1.tm_mday &&
            t1.tm_mon == t2.tm_mon && t1.tm_year == t2.tm_year;
  }

  bool XmlRpcValue::operator==(XmlRpcValue const& other) const
  {
    if (_type != other._type)
      return false;

    switch (_type) {
      case TypeBoolean:  return ( !_value.asBool && !other._value.asBool) ||
                                ( _value.asBool && other._value.asBool);
      case TypeInt:      return _value.asInt == other._value.asInt;
      case TypeDouble:   return _value.asDouble == other._value.asDouble;
      case TypeDateTime: return tmEq(*_value.asTime, *other._value.asTime);
      case TypeString:   return *_value.asString == *other._value.asString;
      case TypeBase64:   return *_value.asBinary == *other._value.asBinary;
      case TypeArray:    return *_value.asArray == *other._value.asArray;

      // The map<>::operator== requires the definition of value< for kcc
      case TypeStruct:   //return *_value.asStruct == *other._value.asStruct;
        {
          if (_value.asStruct->size() != other._value.asStruct->size())
            return false;
          
          ValueStruct::const_iterator it1=_value.asStruct->begin();
          ValueStruct::const_iterator it2=other._value.asStruct->begin();
          while (it1 != _value.asStruct->end()) {
            const XmlRpcValue& v1 = it1->second;
            const XmlRpcValue& v2 = it2->second;
            if ( ! (v1 == v2))
              return false;
            it1++;
            it2++;
          }
          return true;
        }
      default: break;
    }
    return true;    // Both invalid values ...
  }

  bool XmlRpcValue::operator!=(XmlRpcValue const& other) const
  {
    return !(*this == other);
  }


  // Works for strings, binary data, arrays, and structs.
  int XmlRpcValue::size() const
  {
    switch (_type) {
      case TypeString: return int(_value.asString->size());
      case TypeBase64: return int(_value.asBinary->size());
      case TypeArray:  return int(_value.asArray->size());
      case TypeStruct: return int(_value.asStruct->size());
      default: break;
    }

    throw XmlRpcException("type error");
  }

  // Checks for existence of struct member
  bool XmlRpcValue::hasMember(const std::string& name) const
  {
    return _type == TypeStruct && _value.asStruct->find(name) != _value.asStruct->end();
  }

  // Set the value from xml. The chars at *offset into valueXml 
  // should be the start of a <value> tag. Destroys any existing value.
  bool XmlRpcValue::fromXml(std::string const& valueXml, int* offset)
  {
    int savedOffset = *offset;

    invalidate();
    if ( ! XmlRpcUtil::nextTagIs(VALUE_TAG, valueXml, offset))
      return false;       // Not a value, offset not updated

	int afterValueOffset = *offset;
    std::string typeTag = XmlRpcUtil::getNextTag(valueXml, offset);
    bool result = false;
    if (typeTag == BOOLEAN_TAG)
      result = boolFromXml(valueXml, offset);
    else if (typeTag == I4_TAG || typeTag == INT_TAG)
      result = intFromXml(valueXml, offset);
    else if (typeTag == DOUBLE_TAG)
      result = doubleFromXml(valueXml, offset);
    else if (typeTag.empty() || typeTag == STRING_TAG)
      result = stringFromXml(valueXml, offset);
    else if (typeTag == DATETIME_TAG)
      result = timeFromXml(valueXml, offset);
    else if (typeTag == BASE64_TAG)
      result = binaryFromXml(valueXml, offset);
    else if (typeTag == ARRAY_TAG)
      result = arrayFromXml(valueXml, offset);
    else if (typeTag == STRUCT_TAG)
      result = structFromXml(valueXml, offset);
    // Watch for empty/blank strings with no <string>tag
    else if (typeTag == VALUE_ETAG)
    {
      *offset = afterValueOffset;   // back up & try again
      result = stringFromXml(valueXml, offset);
    }

    if (result)  // Skip over the </value> tag
      XmlRpcUtil::findTag(VALUE_ETAG, valueXml, offset);
    else        // Unrecognized tag after <value>
      *offset = savedOffset;

    return result;
  }

  // Encode the Value in xml
  std::string XmlRpcValue::toXml() const
  {
    switch (_type) {
      case TypeBoolean:  return boolToXml();
      case TypeInt:      return intToXml();
      case TypeDouble:   return doubleToXml();
      case TypeString:   return stringToXml();
      case TypeDateTime: return timeToXml();
      case TypeBase64:   return binaryToXml();
      case TypeArray:    return arrayToXml();
      case TypeStruct:   return structToXml();
      default: break;
    }
    return std::string();   // Invalid value
  }


  // Boolean
  bool XmlRpcValue::boolFromXml(std::string const& valueXml, int* offset)
  {
    const char* valueStart = valueXml.c_str() + *offset;
    char* valueEnd;
    long ivalue = strtol(valueStart, &valueEnd, 10);
    if (valueEnd == valueStart || (ivalue != 0 && ivalue != 1))
      return false;

    _type = TypeBoolean;
    _value.asBool = (ivalue == 1);
    *offset += int(valueEnd - valueStart);
    return true;
  }

  std::string XmlRpcValue::boolToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += BOOLEAN_TAG;
    xml += (_value.asBool ? "1" : "0");
    xml += BOOLEAN_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  // Int
  bool XmlRpcValue::intFromXml(std::string const& valueXml, int* offset)
  {
    const char* valueStart = valueXml.c_str() + *offset;
    char* valueEnd;
    long ivalue = strtol(valueStart, &valueEnd, 10);
    if (valueEnd == valueStart)
      return false;

    _type = TypeInt;
    _value.asInt = int(ivalue);
    *offset += int(valueEnd - valueStart);
    return true;
  }

  std::string XmlRpcValue::intToXml() const
  {
    char buf[256];
    snprintf(buf, sizeof(buf)-1, "%d", _value.asInt);
    buf[sizeof(buf)-1] = 0;
    std::string xml = VALUE_TAG;
    xml += I4_TAG;
    xml += buf;
    xml += I4_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  // Double
  bool XmlRpcValue::doubleFromXml(std::string const& valueXml, int* offset)
  {
    const char* valueStart = valueXml.c_str() + *offset;
    char* valueEnd;

    // ticket #2438
    // push/pop the locale here. Value 123.45 can get read by strtod
    // as '123', if the locale expects a comma instead of dot.
    // if there are locale problems, silently continue.
    std::string tmplocale;
    char* locale_cstr = setlocale(LC_NUMERIC, 0);
    if (locale_cstr)
      {
        tmplocale = locale_cstr;
        setlocale(LC_NUMERIC, "POSIX");
      }

    double dvalue = strtod(valueStart, &valueEnd);

    if (tmplocale.size() > 0) {
      setlocale(LC_NUMERIC, tmplocale.c_str());
    }

    if (valueEnd == valueStart)
      return false;

    _type = TypeDouble;
    _value.asDouble = dvalue;
    *offset += int(valueEnd - valueStart);
    return true;
  }

  std::string XmlRpcValue::doubleToXml() const
  {
    // ticket #2438
    std::stringstream ss;
    ss.imbue(std::locale::classic()); // ensure we're using "C" locale for formatting floating-point (1.4 vs. 1,4, etc.)
    ss.precision(17);
    ss << _value.asDouble;

    std::string xml = VALUE_TAG;
    xml += DOUBLE_TAG;
    xml += ss.str();
    xml += DOUBLE_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  // String
  bool XmlRpcValue::stringFromXml(std::string const& valueXml, int* offset)
  {
    size_t valueEnd = valueXml.find('<', *offset);
    if (valueEnd == std::string::npos)
      return false;     // No end tag;

    _type = TypeString;
    _value.asString = new std::string(XmlRpcUtil::xmlDecode(valueXml.substr(*offset, valueEnd-*offset)));
    *offset += int(_value.asString->length());
    return true;
  }

  std::string XmlRpcValue::stringToXml() const
  {
    std::string xml = VALUE_TAG;
    //xml += STRING_TAG; optional
    xml += XmlRpcUtil::xmlEncode(*_value.asString);
    //xml += STRING_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  // DateTime (stored as a struct tm)
  bool XmlRpcValue::timeFromXml(std::string const& valueXml, int* offset)
  {
    size_t valueEnd = valueXml.find('<', *offset);
    if (valueEnd == std::string::npos)
      return false;     // No end tag;

    std::string stime = valueXml.substr(*offset, valueEnd-*offset);

    struct tm t;
#ifdef _MSC_VER
    if (sscanf_s(stime.c_str(),"%4d%2d%2dT%2d:%2d:%2d",&t.tm_year,&t.tm_mon,&t.tm_mday,&t.tm_hour,&t.tm_min,&t.tm_sec) != 6)
#else
    if (sscanf(stime.c_str(),"%4d%2d%2dT%2d:%2d:%2d",&t.tm_year,&t.tm_mon,&t.tm_mday,&t.tm_hour,&t.tm_min,&t.tm_sec) != 6)
#endif
      return false;

    t.tm_isdst = -1;
    _type = TypeDateTime;
    _value.asTime = new struct tm(t);
    *offset += int(stime.length());
    return true;
  }

  std::string XmlRpcValue::timeToXml() const
  {
    struct tm* t = _value.asTime;
    char buf[20];
    snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d", 
      t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
    buf[sizeof(buf)-1] = 0;

    std::string xml = VALUE_TAG;
    xml += DATETIME_TAG;
    xml += buf;
    xml += DATETIME_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }


  // Base64
  bool XmlRpcValue::binaryFromXml(std::string const& valueXml, int* offset)
  {
    size_t valueEnd = valueXml.find('<', *offset);
    if (valueEnd == std::string::npos)
      return false;     // No end tag;

    _type = TypeBase64;
    std::string asString = valueXml.substr(*offset, valueEnd-*offset);
    _value.asBinary = new BinaryData();
    // check whether base64 encodings can contain chars xml encodes...

    // convert from base64 to binary
    int iostatus = 0;
	  base64<char> decoder;
    std::back_insert_iterator<BinaryData> ins = std::back_inserter(*(_value.asBinary));
		decoder.get(asString.begin(), asString.end(), ins, iostatus);

    *offset += int(asString.length());
    return true;
  }


  std::string XmlRpcValue::binaryToXml() const
  {
    // convert to base64
    std::vector<char> base64data;
    int iostatus = 0;
	  base64<char> encoder;
    std::back_insert_iterator<std::vector<char> > ins = std::back_inserter(base64data);
		encoder.put(_value.asBinary->begin(), _value.asBinary->end(), ins, iostatus, base64<>::crlf());

    // Wrap with xml
    std::string xml = VALUE_TAG;
    xml += BASE64_TAG;
    xml.append(base64data.begin(), base64data.end());
    xml += BASE64_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }


  // Array
  bool XmlRpcValue::arrayFromXml(std::string const& valueXml, int* offset)
  {
    if ( ! XmlRpcUtil::nextTagIs(DATA_TAG, valueXml, offset))
      return false;

    _type = TypeArray;
    _value.asArray = new ValueArray;
    XmlRpcValue v;
    while (v.fromXml(valueXml, offset))
      _value.asArray->push_back(v);       // copy...

    // Skip the trailing </data>
    (void) XmlRpcUtil::nextTagIs(DATA_ETAG, valueXml, offset);
    return true;
  }


  // In general, its preferable to generate the xml of each element of the
  // array as it is needed rather than glomming up one big string.
  std::string XmlRpcValue::arrayToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += ARRAY_TAG;
    xml += DATA_TAG;

    int s = int(_value.asArray->size());
    for (int i=0; i<s; ++i)
       xml += _value.asArray->at(i).toXml();

    xml += DATA_ETAG;
    xml += ARRAY_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }


  // Struct
  bool XmlRpcValue::structFromXml(std::string const& valueXml, int* offset)
  {
    _type = TypeStruct;
    _value.asStruct = new ValueStruct;

    while (XmlRpcUtil::nextTagIs(MEMBER_TAG, valueXml, offset)) {
      // name
      const std::string name = XmlRpcUtil::parseTag(NAME_TAG, valueXml, offset);
      // value
      XmlRpcValue val(valueXml, offset);
      if ( ! val.valid()) {
        invalidate();
        return false;
      }
      const std::pair<const std::string, XmlRpcValue> p(name, val);
      _value.asStruct->insert(p);

      (void) XmlRpcUtil::nextTagIs(MEMBER_ETAG, valueXml, offset);
    }
    return true;
  }


  // In general, its preferable to generate the xml of each element
  // as it is needed rather than glomming up one big string.
  std::string XmlRpcValue::structToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += STRUCT_TAG;

    ValueStruct::const_iterator it;
    for (it=_value.asStruct->begin(); it!=_value.asStruct->end(); ++it) {
      xml += MEMBER_TAG;
      xml += NAME_TAG;
      xml += XmlRpcUtil::xmlEncode(it->first);
      xml += NAME_ETAG;
      xml += it->second.toXml();
      xml += MEMBER_ETAG;
    }

    xml += STRUCT_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }



  // Write the value without xml encoding it
  std::ostream& XmlRpcValue::write(std::ostream& os) const {
    switch (_type) {
      default:           break;
      case TypeBoolean:  os << _value.asBool; break;
      case TypeInt:      os << _value.asInt; break;
      case TypeDouble:   os << _value.asDouble; break;
      case TypeString:   os << *_value.asString; break;
      case TypeDateTime:
        {
          struct tm* t = _value.asTime;
          char buf[20];
          snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d", 
            t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
          buf[sizeof(buf)-1] = 0;
          os << buf;
          break;
        }
      case TypeBase64:
        {
          int iostatus = 0;
          std::ostreambuf_iterator<char> out(os);
          base64<char> encoder;
          encoder.put(_value.asBinary->begin(), _value.asBinary->end(), out, iostatus, base64<>::crlf());
          break;
        }
      case TypeArray:
        {
          int s = int(_value.asArray->size());
          os << '{';
          for (int i=0; i<s; ++i)
          {
            if (i > 0) os << ',';
            _value.asArray->at(i).write(os);
          }
          os << '}';
          break;
        }
      case TypeStruct:
        {
          os << '[';
          ValueStruct::const_iterator it;
          for (it=_value.asStruct->begin(); it!=_value.asStruct->end(); ++it)
          {
            if (it!=_value.asStruct->begin()) os << ',';
            os << it->first << ':';
            it->second.write(os);
          }
          os << ']';
          break;
        }
      
    }
    
    return os;
  }

} // namespace XmlRpc


// ostream
std::ostream& operator<<(std::ostream& os, XmlRpc::XmlRpcValue& v) 
{ 
  // If you want to output in xml format:
  //return os << v.toXml(); 
  return v.write(os);
}

