package src.control.loops;

queue_group FlightLoopQueue {
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
  };

  message Status {
    int32_t state;
    int32_t flight_time;
  };

  message Goal {
    bool run_mission;

    bool trigger_failsafe;
    bool trigger_throttle_cut;
  };

  message Output {
    float velocity_x;
    float velocity_y;
    float velocity_z;

    bool velocity_control;
    bool arm;
    bool disarm;
    bool takeoff;
    bool land;
    bool throttle_cut;
  };

  queue Sensors sensors;
  queue Status status;
  queue Goal goal;
  queue Output output;
};

queue_group FlightLoopQueue flight_loop_queue;
