package src.control.loops;

queue_group FlightLoopQueue {
  // Add a dummy message to fix missing hashing function for first queue group
  // entry.
  message Dummy {};

  message Sensors {
    double latitude;
    double longitude;
    float altitude;
    float relative_altitude;
    float heading;

    float velocity_x;
    float velocity_y;
    float velocity_z;

    float gps_ground_speed;

    uint8_t gps_satellite_count;
    uint16_t gps_eph;
    uint16_t gps_epv;

    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float absolute_pressure;
    float relative_pressure;
    float pressure_altitude;
    float temperature;

    float battery_voltage;
    float battery_current;

    bool armed;
    int32_t autopilot_state;

    int32_t state;

    double last_gps;
  };

  message Status {
    int32_t state;
    int32_t flight_time;
    int32_t current_command_index;
  };

  message Goal {
    bool run_mission;

    bool trigger_failsafe;
    bool trigger_throttle_cut;

    double trigger_takeoff;
    double trigger_hold;
    double trigger_offboard;
    double trigger_rtl;
    double trigger_land;
    double trigger_arm;
    double trigger_disarm;

    double trigger_alarm;
    double trigger_bomb_drop;
  };

  message Output {
    float velocity_x;
    float velocity_y;
    float velocity_z;

    float gimbal_angle;
    bool bomb_drop;

    double trigger_takeoff;
    double trigger_hold;
    double trigger_offboard;
    double trigger_rtl;
    double trigger_land;
    double trigger_arm;
    double trigger_disarm;

    bool alarm;
  };

  queue Dummy dummy;
  queue Sensors sensors;
  queue Status status;
  queue Goal goal;
  queue Output output;
};

queue_group FlightLoopQueue flight_loop_queue;
