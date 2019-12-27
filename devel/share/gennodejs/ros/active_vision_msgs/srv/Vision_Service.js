// Auto-generated. Do not edit!

// (in-package active_vision_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let Vision_Message = require('../msg/Vision_Message.js');

//-----------------------------------------------------------

class Vision_ServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Label = null;
    }
    else {
      if (initObj.hasOwnProperty('Label')) {
        this.Label = initObj.Label
      }
      else {
        this.Label = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vision_ServiceRequest
    // Serialize message field [Label]
    bufferOffset = _serializer.string(obj.Label, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vision_ServiceRequest
    let len;
    let data = new Vision_ServiceRequest(null);
    // Deserialize message field [Label]
    data.Label = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Label.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'active_vision_msgs/Vision_ServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec13f5aded047f0ac89d028f3701bc0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Label
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Vision_ServiceRequest(null);
    if (msg.Label !== undefined) {
      resolved.Label = msg.Label;
    }
    else {
      resolved.Label = ''
    }

    return resolved;
    }
};

class Vision_ServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_location = null;
    }
    else {
      if (initObj.hasOwnProperty('object_location')) {
        this.object_location = initObj.object_location
      }
      else {
        this.object_location = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vision_ServiceResponse
    // Serialize message field [object_location]
    // Serialize the length for message field [object_location]
    bufferOffset = _serializer.uint32(obj.object_location.length, buffer, bufferOffset);
    obj.object_location.forEach((val) => {
      bufferOffset = Vision_Message.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vision_ServiceResponse
    let len;
    let data = new Vision_ServiceResponse(null);
    // Deserialize message field [object_location]
    // Deserialize array length for message field [object_location]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.object_location = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.object_location[i] = Vision_Message.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.object_location.forEach((val) => {
      length += Vision_Message.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'active_vision_msgs/Vision_ServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c95021b1df9fae604953565f51cf488';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Vision_Message[] object_location
    
    
    ================================================================================
    MSG: active_vision_msgs/Vision_Message
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
    const resolved = new Vision_ServiceResponse(null);
    if (msg.object_location !== undefined) {
      resolved.object_location = new Array(msg.object_location.length);
      for (let i = 0; i < resolved.object_location.length; ++i) {
        resolved.object_location[i] = Vision_Message.Resolve(msg.object_location[i]);
      }
    }
    else {
      resolved.object_location = []
    }

    return resolved;
    }
};

module.exports = {
  Request: Vision_ServiceRequest,
  Response: Vision_ServiceResponse,
  md5sum() { return '743c237c8927abfed6464a4d17c2f4fc'; },
  datatype() { return 'active_vision_msgs/Vision_Service'; }
};
