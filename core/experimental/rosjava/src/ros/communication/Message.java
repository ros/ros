/*
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
 */

// author: Jason Wolfe


package ros.communication;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public abstract class Message implements Cloneable {
	public static String __s_getDataType() { throw new UnsupportedOperationException(); }
	public static String __s_getMD5Sum() { throw new UnsupportedOperationException(); }
	public static String __s_getMessageDefinition() { throw new UnsupportedOperationException(); }	
	
	public abstract String getDataType();
	public abstract String getMD5Sum();
	public String getServerMD5Sum() {throw new UnsupportedOperationException();}
	public abstract String getMessageDefinition();

	public abstract int    serializationLength();
	public abstract void   serialize(OutputStream os, int seq) throws IOException;
	public abstract void   deserialize(InputStream is)         throws IOException;

	public byte [] serialize(int seq) {
		try {
			int len = serializationLength();
			ByteArrayOutputStream baos = new ByteArrayOutputStream(serializationLength());
			serialize(baos, seq);
			byte [] ret = baos.toByteArray();
			if (ret.length != len) throw new RuntimeException("Non-matching serialization length!");
//			System.out.println("Wrote " + ret.length + " bytes: ");
//			for(int i = 0; i < ret.length; i++) System.out.format("%x,", ret[i]);
			return ret;
		} catch (IOException e) {
			throw new RuntimeException("Error in serialize?!"); // Should never happen
		}		
	}
	
	public void deserialize(byte [] data) {
		try {
//			System.out.println("Read " + data.length + " bytes: ");
//			for(int i = 0; i < Math.min(100, data.length); i++) System.out.format("%x,", data[i]);
			deserialize(new ByteArrayInputStream(data));
		} catch (IOException e) {
			throw new RuntimeException("Error in deserialize?!"); // Should never happen
		}
	}
		
	public Message clone() {
		try {
			return (Message) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException("Clone of message not supported?!");
		}
	}
	
	public abstract void setTo(Message m);
	
    // Unfortunately, Java DataOutput is Big Endian and messages are Little Endian...
	// TODO: java nio ByteBuffers support changing endian-ness.
	public static class Serialization {
		public static byte readByte(InputStream is) throws IOException {
			int ret = 0;
			ret |= ((0xFF & is.read()) << 0);
			return (byte) ret;
		}

		public static short readShort(InputStream is) throws IOException {
			int ret = 0;
			ret |= ((0xFF & is.read()) << 0);
			ret |= ((0xFF & is.read()) << 8);
			return (short) ret;
		}

		public static int readInt(InputStream is) throws IOException {
			int ret = 0;
			ret |= ((0xFF & is.read()) << 0);
			ret |= ((0xFF & is.read()) << 8);
			ret |= ((0xFF & is.read()) << 16);
			ret |= ((0xFF & is.read()) << 24);
			return (int) ret;
		}

		public static long readLong(InputStream is) throws IOException {
			long ret = 0;
			ret |= ((0xFFL & is.read()) << 0);
			ret |= ((0xFFL & is.read()) << 8);
			ret |= ((0xFFL & is.read()) << 16);
			ret |= ((0xFFL & is.read()) << 24);
			ret |= ((0xFFL & is.read()) << 32);
			ret |= ((0xFFL & is.read()) << 40);
			ret |= ((0xFFL & is.read()) << 48);
			ret |= ((0xFFL & is.read()) << 56);
			return (long) ret;
		}

		public static float readFloat(InputStream is) throws IOException {
			return Float.intBitsToFloat(readInt(is));
		}

		public static double readDouble(InputStream is) throws IOException {
			return Double.longBitsToDouble(readLong(is));
		}
		
		public static String readString(InputStream is) throws IOException {
			int len = readInt(is);
			byte [] bytes = new byte[len];
			int read = is.read(bytes);
			if (read != len) throw new RuntimeException("Bad string length");
			return new String(bytes);
		}

		public static Time readTime(InputStream is) throws IOException {
			Time t = new Time(readInt(is), readInt(is));
			return t;
		}

		public static Duration readDuration(InputStream is) throws IOException {
			Duration t = new Duration(readInt(is), readInt(is));
			return t;
		}
		
		
		
		public static void writeByte(OutputStream os, byte o) throws IOException {
			os.write(o >>> 0);
		}

		public static void writeShort(OutputStream os, short o) throws IOException {
			os.write(o >>> 0);
			os.write(o >>> 8);
		}

		public static void writeInt(OutputStream os, int o) throws IOException {
			os.write(o >>> 0);
			os.write(o >>> 8);
			os.write(o >>> 16);
			os.write(o >>> 24);
		}

		public static void writeLong(OutputStream os, long o) throws IOException {
			os.write((int) (o >>> 0));
			os.write((int) (o >>> 8));
			os.write((int) (o >>> 16));
			os.write((int) (o >>> 24));
			os.write((int) (o >>> 32));
			os.write((int) (o >>> 40));
			os.write((int) (o >>> 48));
			os.write((int) (o >>> 56));
		}

		public static void writeFloat(OutputStream os, float o) throws IOException {
			writeInt(os, Float.floatToRawIntBits(o));
		}

		public static void writeDouble(OutputStream os, double o) throws IOException  {
			writeLong(os, Double.doubleToRawLongBits(o));
		}

		public static void writeString(OutputStream os, String s) throws IOException {
			writeInt(os, s.length());
			os.write(s.getBytes());
		}
		
		public static void writeTime(OutputStream os, Time o) throws IOException  {
			writeInt(os,o.secs);
			writeInt(os,o.nsecs);
		}

		public static void writeDuration(OutputStream os, Duration o) throws IOException  {
			writeInt(os,o.secs);
			writeInt(os,o.nsecs);
		}
	}
}
