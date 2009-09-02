# Test all primitive types
byte     byte_
char     char_
uint8    uint8_
int8     int8_
uint16   uint16_
int16    int16_
uint32   uint32_
int32    int32_
uint64   uint64_
int64    int64_
float32  float32_
float64  float64_
string   string_
time     time_
duration duration_

# Test a smattering of array types
byte[]     byte_v
byte[2]    byte_f
float64[]  float64_v
float64[2] float64_f
string[]   string_v
string[2]  string_f
time[]     time_v
time[2]    time_f

# Test submsgs, including both fixed and var length
std_msgs/Byte    Byte_
std_msgs/Byte[]  Byte_v

std_msgs/ByteMultiArray    ByteMultiArray_
std_msgs/ByteMultiArray[]  ByteMultiArray_v

# Unfortunately, can't test these because roscpp message generation
# is broken.  Hopefully rosjava works correctly ...
# TODO: put these back in.

# std_msgs/Byte[2] Byte_f
# std_msgs/ByteMultiArray[2] ByteMultiArray_f
