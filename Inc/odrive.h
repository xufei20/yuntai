#ifndef __ODRIVE_H
#define __ODRIVE_H

#define AXIS0_NODE_ID  (1<<5) // odrive ID
#define AXIS1_NODE_ID  (2<<5)

typedef union _HeartBeat_Box
{
  uint8_t data[8];
  struct _
  {
    float angle;
		float average_speed;
//    float axis0_speed;
//    float axis1_speed;
  }value;
}HeartBeat_Box_T;

typedef union _float_to_uint8_t
{
  uint8_t raw[8];
  float value[2];
  uint32_t u32_data[2];
  int32_t int32_data[2];
}float_to_uint8_t;

typedef enum
{
    AXIS_0 = 0,
    AXIS_1 = 1
} Axis_t;

typedef enum
{
    AXIS_STATE_UNDEFINED = 0,                  //<! will fall through to idle
    AXIS_STATE_IDLE = 1,                       //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2,           //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,  //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4,          //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5,         //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,       //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,        //<! run closed loop control
    AXIS_STATE_LOCKIN_SPIN = 9,                //<! run lockin spin
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING = 11, //<! run axis homing function
} State_t;


typedef struct
{
    uint32_t axis_error;
    uint32_t axis_current_state;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    float encoder_pos_estimate;
    float encoder_vel_estimate;
    int32_t encoder_shadow_count;
    int32_t encoder_cpr_count;
    float iq_setpoint;
    float iq_measured;
    float sensorless_pos_estimate;
    float sensorless_vel_estimate;
    float vbus_voltage;
} OdriveAxisGetState_t;

typedef struct
{
    uint16_t axis_node_id;
    uint32_t requested_state;
    int32_t control_mode;
    int32_t input_mode;
    int16_t vel_ff;
    int16_t current_ff;
    int32_t input_pos;
    // int32_t input_vel;
    float input_vel;
    float torque_vel;
    int32_t input_current;
    float vel_limit;
    float traj_vel_limit;
    float traj_accel_limit;
    float traj_decel_limit;
    float traj_a_per_css;
} OdriveAxisSetState_t;

typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR, // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL = 0x00D,
    MSG_SET_INPUT_CURRENT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} Odrive_Commond;

enum OdriveControllerMode {
  ODRV_CTRL_INPUT_MODE_DISABLE = 0,
  ODRV_CTRL_INPUT_MODE_PASSTHROUGH = 1,
  ODRV_CTRL_INPUT_MODE_VEL_RAMP = 2,
  ODRV_CTRL_INPUT_MODE_POS_FILTER = 3,
  ODRV_CTRL_INPUT_MODE_TRAP_TRAJ = 5,
  ODRV_CTRL_INPUT_MODE_TOQ_RAMP = 6
};

enum OdriveControllerCtrlMode{
  ODRV_CTRL_CTRL_MODE_VOLTAGE = 0,
  ODRV_CTRL_CTRL_MODE_TORQ = 1,
  ODRV_CTRL_CTRL_MODE_VEL = 2,
  ODRV_CTRL_CTRL_MODE_POS = 3
};

enum OdriveReqState {
  ODRV_REQ_STA_UNDEF = 0,
  ODRV_REQ_STA_IDEL = 1,
  ODRV_REQ_STA_FULL_CALIB = 3,
  ODRV_REQ_STA_CLOSE_LOOP_CTRL = 8,
};


uint8_t odrv_write_msg(Axis_t axis, Odrive_Commond cmd);


#endif



