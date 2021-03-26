// Auto-generated. Do not edit!

// (in-package ur3_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gripper_input {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.DIGIN = null;
      this.AIN0 = null;
      this.AIN1 = null;
    }
    else {
      if (initObj.hasOwnProperty('DIGIN')) {
        this.DIGIN = initObj.DIGIN
      }
      else {
        this.DIGIN = 0;
      }
      if (initObj.hasOwnProperty('AIN0')) {
        this.AIN0 = initObj.AIN0
      }
      else {
        this.AIN0 = 0.0;
      }
      if (initObj.hasOwnProperty('AIN1')) {
        this.AIN1 = initObj.AIN1
      }
      else {
        this.AIN1 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gripper_input
    // Serialize message field [DIGIN]
    bufferOffset = _serializer.int32(obj.DIGIN, buffer, bufferOffset);
    // Serialize message field [AIN0]
    bufferOffset = _serializer.float64(obj.AIN0, buffer, bufferOffset);
    // Serialize message field [AIN1]
    bufferOffset = _serializer.float64(obj.AIN1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gripper_input
    let len;
    let data = new gripper_input(null);
    // Deserialize message field [DIGIN]
    data.DIGIN = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [AIN0]
    data.AIN0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [AIN1]
    data.AIN1 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ur3_driver/gripper_input';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4b7d373885d48c37baffd91ce2f1c38';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 DIGIN 
    float64 AIN0
    float64 AIN1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gripper_input(null);
    if (msg.DIGIN !== undefined) {
      resolved.DIGIN = msg.DIGIN;
    }
    else {
      resolved.DIGIN = 0
    }

    if (msg.AIN0 !== undefined) {
      resolved.AIN0 = msg.AIN0;
    }
    else {
      resolved.AIN0 = 0.0
    }

    if (msg.AIN1 !== undefined) {
      resolved.AIN1 = msg.AIN1;
    }
    else {
      resolved.AIN1 = 0.0
    }

    return resolved;
    }
};

module.exports = gripper_input;
