// Auto-generated. Do not edit!

// (in-package active_perception.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Vision_Message {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Frameid = null;
      this.Pos = null;
      this.Found = null;
    }
    else {
      if (initObj.hasOwnProperty('Frameid')) {
        this.Frameid = initObj.Frameid
      }
      else {
        this.Frameid = '';
      }
      if (initObj.hasOwnProperty('Pos')) {
        this.Pos = initObj.Pos
      }
      else {
        this.Pos = new geometry_msgs.msg.Point32();
      }
      if (initObj.hasOwnProperty('Found')) {
        this.Found = initObj.Found
      }
      else {
        this.Found = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vision_Message
    // Serialize message field [Frameid]
    bufferOffset = _serializer.string(obj.Frameid, buffer, bufferOffset);
    // Serialize message field [Pos]
    bufferOffset = geometry_msgs.msg.Point32.serialize(obj.Pos, buffer, bufferOffset);
    // Serialize message field [Found]
    bufferOffset = _serializer.bool(obj.Found, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vision_Message
    let len;
    let data = new Vision_Message(null);
    // Deserialize message field [Frameid]
    data.Frameid = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [Pos]
    data.Pos = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset);
    // Deserialize message field [Found]
    data.Found = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Frameid.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'active_perception/Vision_Message';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e12f5951169920222965062c93aead0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Frameid
    geometry_msgs/Point32 Pos
    bool Found
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Vision_Message(null);
    if (msg.Frameid !== undefined) {
      resolved.Frameid = msg.Frameid;
    }
    else {
      resolved.Frameid = ''
    }

    if (msg.Pos !== undefined) {
      resolved.Pos = geometry_msgs.msg.Point32.Resolve(msg.Pos)
    }
    else {
      resolved.Pos = new geometry_msgs.msg.Point32()
    }

    if (msg.Found !== undefined) {
      resolved.Found = msg.Found;
    }
    else {
      resolved.Found = false
    }

    return resolved;
    }
};

module.exports = Vision_Message;
