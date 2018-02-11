package spinny.control.loops;

queue_group FlightLoopQueue {
  message Sensors {
    float latitude;
    float longitude;
    float altitude;
    float relative_altitude;

    float velocity_x;
    float velocity_y;
    float velocity_z;

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
  };

  message Output {
    float velocity_x;
    float velocity_y;
    float velocity_z;

    bool arm;
    bool takeoff;
    bool land;
    bool throttle_cut;
  };

  queue Sensors sensors;
  queue Output output;
};

queue_group FlightLoopQueue flight_loop_queue;
