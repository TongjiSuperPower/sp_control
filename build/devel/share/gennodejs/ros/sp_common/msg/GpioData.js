// Auto-generated. Do not edit!

// (in-package sp_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GpioData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bus = null;
      this.gpio_name = null;
      this.gpio_state = null;
      this.gpio_type = null;
      this.header = null;
    }
    else {
      if (initObj.hasOwnProperty('bus')) {
        this.bus = initObj.bus
      }
      else {
        this.bus = [];
      }
      if (initObj.hasOwnProperty('gpio_name')) {
        this.gpio_name = initObj.gpio_name
      }
      else {
        this.gpio_name = [];
      }
      if (initObj.hasOwnProperty('gpio_state')) {
        this.gpio_state = initObj.gpio_state
      }
      else {
        this.gpio_state = [];
      }
      if (initObj.hasOwnProperty('gpio_type')) {
        this.gpio_type = initObj.gpio_type
      }
      else {
        this.gpio_type = [];
      }
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioData
    // Serialize message field [bus]
    bufferOffset = _arraySerializer.string(obj.bus, buffer, bufferOffset, null);
    // Serialize message field [gpio_name]
    bufferOffset = _arraySerializer.string(obj.gpio_name, buffer, bufferOffset, null);
    // Serialize message field [gpio_state]
    bufferOffset = _arraySerializer.bool(obj.gpio_state, buffer, bufferOffset, null);
    // Serialize message field [gpio_type]
    bufferOffset = _arraySerializer.string(obj.gpio_type, buffer, bufferOffset, null);
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioData
    let len;
    let data = new GpioData(null);
    // Deserialize message field [bus]
    data.bus = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [gpio_name]
    data.gpio_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [gpio_state]
    data.gpio_state = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [gpio_type]
    data.gpio_type = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.bus.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.gpio_name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += object.gpio_state.length;
    object.gpio_type.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sp_common/GpioData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '297858866c40f438698ae2b119d8be46';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] bus
    string[] gpio_name
    bool[] gpio_state
    string[] gpio_type
    std_msgs/Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioData(null);
    if (msg.bus !== undefined) {
      resolved.bus = msg.bus;
    }
    else {
      resolved.bus = []
    }

    if (msg.gpio_name !== undefined) {
      resolved.gpio_name = msg.gpio_name;
    }
    else {
      resolved.gpio_name = []
    }

    if (msg.gpio_state !== undefined) {
      resolved.gpio_state = msg.gpio_state;
    }
    else {
      resolved.gpio_state = []
    }

    if (msg.gpio_type !== undefined) {
      resolved.gpio_type = msg.gpio_type;
    }
    else {
      resolved.gpio_type = []
    }

    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    return resolved;
    }
};

module.exports = GpioData;
