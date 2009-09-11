byte int

# primitive types
byte b
uint8 u8
uint16 u16
uint32 u32
uint64 u64
int8 i8
int16 i16
int32 i32
int64 i64
float32 f32
float64 f64
string str

# "special" primitive types
time ts
duration d

# "special" complex type
Header header

# complex types
std_msgs/Pose2DFloat32 pose

# fixed-length arrays of primitives
byte[8] farr

# variable-length arrays of primitives
byte[] varr

# fixed-length arrays of complex types
std_msgs/Pose2DFloat32[8] farrcomp

# variable-length arrays of complex types
std_msgs/Pose2DFloat32[] varrcomp
