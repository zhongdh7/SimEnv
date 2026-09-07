// Auto-generated. Do not edit!

// (in-package hazard_perception.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Hazard3D = require('./Hazard3D.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Hazard3DArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.hazards = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('hazards')) {
        this.hazards = initObj.hazards
      }
      else {
        this.hazards = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Hazard3DArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [hazards]
    // Serialize the length for message field [hazards]
    bufferOffset = _serializer.uint32(obj.hazards.length, buffer, bufferOffset);
    obj.hazards.forEach((val) => {
      bufferOffset = Hazard3D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Hazard3DArray
    let len;
    let data = new Hazard3DArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [hazards]
    // Deserialize array length for message field [hazards]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.hazards = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.hazards[i] = Hazard3D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.hazards.forEach((val) => {
      length += Hazard3D.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hazard_perception/Hazard3DArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45285fb7a9658ae251ee2d8e7e727d24';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    Hazard3D[] hazards
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: hazard_perception/Hazard3D
    uint32 id
    geometry_msgs/Point position
    float32 confidence
    uint32 observation_count
    bool confirmed
    string localization_source
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Hazard3DArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.hazards !== undefined) {
      resolved.hazards = new Array(msg.hazards.length);
      for (let i = 0; i < resolved.hazards.length; ++i) {
        resolved.hazards[i] = Hazard3D.Resolve(msg.hazards[i]);
      }
    }
    else {
      resolved.hazards = []
    }

    return resolved;
    }
};

module.exports = Hazard3DArray;
