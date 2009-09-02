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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public abstract class Message implements Cloneable {
	public static String __s_getDataType() { throw new UnsupportedOperationException(); }
	public static String __s_getMD5Sum() { throw new UnsupportedOperationException(); }
	public static String __s_getMessageDefinition() { throw new UnsupportedOperationException(); }	
	
	public abstract String getDataType();
	public abstract String getMD5Sum();
	public String getServerMD5Sum() {throw new UnsupportedOperationException();}
	public abstract String getMessageDefinition();

	public abstract int    serializationLength();
	public abstract void   serialize(ByteBuffer bb, int seq);
	public abstract void   deserialize(ByteBuffer bb);

	public byte [] serialize(int seq) {
		int len = serializationLength();
		ByteBuffer bb = ByteBuffer.allocate(len).order(ByteOrder.LITTLE_ENDIAN);
		serialize(bb, seq);
		byte [] ret = bb.array();
		if (ret.length != len) throw new RuntimeException("Non-matching serialization length!");
		//			System.out.println("Wrote " + ret.length + " bytes: ");
		//			for(int i = 0; i < ret.length; i++) System.out.format("%x,", ret[i]);
		return ret;
	}
	
	public void deserialize(byte [] data) {
		//			System.out.println("Read " + data.length + " bytes: ");
		//			for(int i = 0; i < Math.min(100, data.length); i++) System.out.format("%x,", data[i]);
		ByteBuffer bb = ByteBuffer.wrap(data);
		deserialize(bb.asReadOnlyBuffer().order(ByteOrder.LITTLE_ENDIAN));
	}
		
	public Message clone() {
		try {
			return (Message) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException("Clone of message not supported?!");
		}
	}
	
	public abstract void setTo(Message m);
	
	public static class Serialization {
		public static String readString(ByteBuffer bb) {
			int len = bb.getInt();
			byte[]  bytes = new byte[len];
			bb.get(bytes);
			return new String(bytes);
		}

		public static Time readTime(ByteBuffer bb)  {
			Time t = new Time(bb.getInt(), bb.getInt());
			return t;
		}

		public static Duration readDuration(ByteBuffer bb)  {
			Duration t = new Duration(bb.getInt(), bb.getInt());
			return t;
		}

		
		public static void writeString(ByteBuffer bb, String s)  {
			byte [] bytes = s.getBytes();
			bb.putInt(bytes.length);
			bb.put(bytes);
		}
		
		public static void writeTime(ByteBuffer bb, Time o)   {
			bb.putInt(o.secs);
			bb.putInt(o.nsecs);
		}

		public static void writeDuration(ByteBuffer bb, Duration o)   {
			bb.putInt(o.secs);
			bb.putInt(o.nsecs);
		}
	}
}
