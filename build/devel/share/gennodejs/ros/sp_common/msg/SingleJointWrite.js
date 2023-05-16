// Auto-generated. Do not edit!

// (in-package sp_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SingleJointWrite {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.num = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0.0;
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SingleJointWrite
    // Serialize message field [state]
    bufferOffset = _serializer.float64(obj.state, buffer, bufferOffset);
    // Serialize message field [num]
    bufferOffset = _serializer.uint16(obj.num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SingleJointWrite
    let len;
    let data = new SingleJointWrite(null);
    // Deserialize message field [state]
    data.state = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [num]
    data.num = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sp_common/SingleJointWrite';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8df87d7f5c9d7fe4d040df6e38c801b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message is for....
    
    float64 state
    uint16 num
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SingleJointWrite(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0.0
    }

    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    return resolved;
    }
};

module.exports = SingleJointWrite;
