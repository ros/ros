% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = test_ros_AddTwoInts()
if( nargout > 0 )
    reqmsg = test_ros_Request();
end
if( nargout > 0 )
    resmsg = test_ros_Response();
end

% Auto-generated.  Do not edit!

% msg = test_ros_Request()
%
% Request message type, fields include:
% int64 a
% int64 b

% //! \htmlinclude Request.msg.html
function msg = test_ros_Request()

msg = [];
msg.create_response_ = @test_ros_Response;
msg.a = int64(0);
msg.b = int64(0);
msg.md5sum_ = @test_ros_Request___md5sum;
msg.server_md5sum_ = @test_ros_Request___server_md5sum;
msg.type_ = @test_ros_Request___type;
msg.serializationLength_ = @test_ros_Request___serializationLength;
msg.serialize_ = @test_ros_Request___serialize;
msg.deserialize_ = @test_ros_Request___deserialize;

function x = test_ros_Request___md5sum()
x = '';

function x = test_ros_Request___server_md5sum()
x = '6a2e34150c00229791cc89ff309fff21';

function x = test_ros_Request___type()
x = 'test_ros/AddTwoIntsRequest';

function l__ = test_ros_Request___serializationLength(msg)
l__ =  ...
    + 8 ...
    + 8;

function dat__ = test_ros_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.a, 'int64');
c__ = c__ + fwrite(fid__, msg__.b, 'int64');
if( c__ ~= 2 )
    error('some members of msg test_ros:Request are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = test_ros_Request___deserialize(dat__, fid__)
msg__ = test_ros_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.a = fread(fid__,1,'int64=>int64');
msg__.b = fread(fid__,1,'int64=>int64');
if( file_created__ )
    fclose(fid__);
end
function l__ = test_ros_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = test_ros_Response()
%
% Response message type, fields include:
% int64 sum

% //! \htmlinclude Response.msg.html
function msg = test_ros_Response()

msg = [];
msg.sum = int64(0);
msg.md5sum_ = @test_ros_Response___md5sum;
msg.server_md5sum_ = @test_ros_Response___server_md5sum;
msg.type_ = @test_ros_Response___type;
msg.serializationLength_ = @test_ros_Response___serializationLength;
msg.serialize_ = @test_ros_Response___serialize;
msg.deserialize_ = @test_ros_Response___deserialize;

function x = test_ros_Response___md5sum()
x = '';

function x = test_ros_Response___server_md5sum()
x = '6a2e34150c00229791cc89ff309fff21';

function x = test_ros_Response___type()
x = 'test_ros/AddTwoIntsResponse';

function l__ = test_ros_Response___serializationLength(msg)
l__ =  ...
    + 8;

function dat__ = test_ros_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.sum, 'int64');
if( c__ ~= 1 )
    error('some members of msg test_ros:Response are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = test_ros_Response___deserialize(dat__, fid__)
msg__ = test_ros_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.sum = fread(fid__,1,'int64=>int64');
if( file_created__ )
    fclose(fid__);
end
function l__ = test_ros_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

