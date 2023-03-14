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

class ActuatorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.name = null;
      this.type = null;
      this.bus = null;
      this.id = null;
      this.halted = null;
      this.position_raw = null;
      this.velocity_raw = null;
      this.circle = null;
      this.last_position_raw = null;
      this.position = null;
      this.velocity = null;
      this.effort = null;
      this.cmd_effort = null;
      this.exe_effort = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = [];
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = [];
      }
      if (initObj.hasOwnProperty('bus')) {
        this.bus = initObj.bus
      }
      else {
        this.bus = [];
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = [];
      }
      if (initObj.hasOwnProperty('halted')) {
        this.halted = initObj.halted
      }
      else {
        this.halted = [];
      }
      if (initObj.hasOwnProperty('position_raw')) {
        this.position_raw = initObj.position_raw
      }
      else {
        this.position_raw = [];
      }
      if (initObj.hasOwnProperty('velocity_raw')) {
        this.velocity_raw = initObj.velocity_raw
      }
      else {
        this.velocity_raw = [];
      }
      if (initObj.hasOwnProperty('circle')) {
        this.circle = initObj.circle
      }
      else {
        this.circle = [];
      }
      if (initObj.hasOwnProperty('last_position_raw')) {
        this.last_position_raw = initObj.last_position_raw
      }
      else {
        this.last_position_raw = [];
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = [];
      }
      if (initObj.hasOwnProperty('cmd_effort')) {
        this.cmd_effort = initObj.cmd_effort
      }
      else {
        this.cmd_effort = [];
      }
      if (initObj.hasOwnProperty('exe_effort')) {
        this.exe_effort = initObj.exe_effort
      }
      else {
        this.exe_effort = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorState
    // Serialize message field [stamp]
    bufferOffset = _arraySerializer.time(obj.stamp, buffer, bufferOffset, null);
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [type]
    bufferOffset = _arraySerializer.string(obj.type, buffer, bufferOffset, null);
    // Serialize message field [bus]
    bufferOffset = _arraySerializer.string(obj.bus, buffer, bufferOffset, null);
    // Serialize message field [id]
    bufferOffset = _arraySerializer.int32(obj.id, buffer, bufferOffset, null);
    // Serialize message field [halted]
    bufferOffset = _arraySerializer.bool(obj.halted, buffer, bufferOffset, null);
    // Serialize message field [position_raw]
    bufferOffset = _arraySerializer.uint16(obj.position_raw, buffer, bufferOffset, null);
    // Serialize message field [velocity_raw]
    bufferOffset = _arraySerializer.int16(obj.velocity_raw, buffer, bufferOffset, null);
    // Serialize message field [circle]
    bufferOffset = _arraySerializer.int64(obj.circle, buffer, bufferOffset, null);
    // Serialize message field [last_position_raw]
    bufferOffset = _arraySerializer.uint16(obj.last_position_raw, buffer, bufferOffset, null);
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float64(obj.position, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [effort]
    bufferOffset = _arraySerializer.float64(obj.effort, buffer, bufferOffset, null);
    // Serialize message field [cmd_effort]
    bufferOffset = _arraySerializer.float64(obj.cmd_effort, buffer, bufferOffset, null);
    // Serialize message field [exe_effort]
    bufferOffset = _arraySerializer.float64(obj.exe_effort, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorState
    let len;
    let data = new ActuatorState(null);
    // Deserialize message field [stamp]
    data.stamp = _arrayDeserializer.time(buffer, bufferOffset, null)
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [type]
    data.type = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [bus]
    data.bus = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [id]
    data.id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [halted]
    data.halted = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [position_raw]
    data.position_raw = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [velocity_raw]
    data.velocity_raw = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [circle]
    data.circle = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [last_position_raw]
    data.last_position_raw = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort]
    data.effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmd_effort]
    data.cmd_effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [exe_effort]
    data.exe_effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.stamp.length;
    object.name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.type.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.bus.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 4 * object.id.length;
    length += object.halted.length;
    length += 2 * object.position_raw.length;
    length += 2 * object.velocity_raw.length;
    length += 8 * object.circle.length;
    length += 2 * object.last_position_raw.length;
    length += 8 * object.position.length;
    length += 8 * object.velocity.length;
    length += 8 * object.effort.length;
    length += 8 * object.cmd_effort.length;
    length += 8 * object.exe_effort.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sp_common/ActuatorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40a5a2d346a26f8d3a3f5c992d8174b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message is for visualizing the ActuatorState
    # The data from real Actuator will be parsed by sp_hw
    # notice that this is a vector of ActuatorState
    
    time[] stamp
    string[] name
    string[] type
    string[] bus
    int32[] id
    bool[] halted
    
    # the direct feedback from rotor's encoder
    uint16[] position_raw
    int16[] velocity_raw
    int64[] circle
    uint16[] last_position_raw
    
    float64[] position
    float64[] velocity
    float64[] effort
    
    
    float64[] cmd_effort 
    float64[] exe_effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActuatorState(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = []
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = []
    }

    if (msg.bus !== undefined) {
      resolved.bus = msg.bus;
    }
    else {
      resolved.bus = []
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = []
    }

    if (msg.halted !== undefined) {
      resolved.halted = msg.halted;
    }
    else {
      resolved.halted = []
    }

    if (msg.position_raw !== undefined) {
      resolved.position_raw = msg.position_raw;
    }
    else {
      resolved.position_raw = []
    }

    if (msg.velocity_raw !== undefined) {
      resolved.velocity_raw = msg.velocity_raw;
    }
    else {
      resolved.velocity_raw = []
    }

    if (msg.circle !== undefined) {
      resolved.circle = msg.circle;
    }
    else {
      resolved.circle = []
    }

    if (msg.last_position_raw !== undefined) {
      resolved.last_position_raw = msg.last_position_raw;
    }
    else {
      resolved.last_position_raw = []
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = []
    }

    if (msg.cmd_effort !== undefined) {
      resolved.cmd_effort = msg.cmd_effort;
    }
    else {
      resolved.cmd_effort = []
    }

    if (msg.exe_effort !== undefined) {
      resolved.exe_effort = msg.exe_effort;
    }
    else {
      resolved.exe_effort = []
    }

    return resolved;
    }
};

module.exports = ActuatorState;
