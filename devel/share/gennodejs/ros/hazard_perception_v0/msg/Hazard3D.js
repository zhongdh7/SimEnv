// Auto-generated. Do not edit!

// (in-package hazard_perception_v0.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Hazard3D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.position = null;
      this.confidence = null;
      this.observation_count = null;
      this.confirmed = null;
      this.localization_source = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('observation_count')) {
        this.observation_count = initObj.observation_count
      }
      else {
        this.observation_count = 0;
      }
      if (initObj.hasOwnProperty('confirmed')) {
        this.confirmed = initObj.confirmed
      }
      else {
        this.confirmed = false;
      }
      if (initObj.hasOwnProperty('localization_source')) {
        this.localization_source = initObj.localization_source
      }
      else {
        this.localization_source = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Hazard3D
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [observation_count]
    bufferOffset = _serializer.uint32(obj.observation_count, buffer, bufferOffset);
    // Serialize message field [confirmed]
    bufferOffset = _serializer.bool(obj.confirmed, buffer, bufferOffset);
    // Serialize message field [localization_source]
    bufferOffset = _serializer.string(obj.localization_source, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Hazard3D
    let len;
    let data = new Hazard3D(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [observation_count]
    data.observation_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [confirmed]
    data.confirmed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [localization_source]
    data.localization_source = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.localization_source);
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hazard_perception_v0/Hazard3D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2af06c5e740031a3e6b4af89771e5573';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Hazard3D(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.observation_count !== undefined) {
      resolved.observation_count = msg.observation_count;
    }
    else {
      resolved.observation_count = 0
    }

    if (msg.confirmed !== undefined) {
      resolved.confirmed = msg.confirmed;
    }
    else {
      resolved.confirmed = false
    }

    if (msg.localization_source !== undefined) {
      resolved.localization_source = msg.localization_source;
    }
    else {
      resolved.localization_source = ''
    }

    return resolved;
    }
};

module.exports = Hazard3D;
