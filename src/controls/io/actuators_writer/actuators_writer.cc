#include "actuators_writer.h"

namespace src {
namespace controls {
namespace io {
namespace actuators_writer {

ActuatorsWriter::ActuatorsWriter() :
    writer_thread_(&ActuatorsWriter::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency),
    alarm_(kWriterPhasedLoopFrequency),
    alarm_subscriber_(ros_node_handle_.subscribe(
        "/uasatucla/actuators/alarm", 1,
        &ActuatorsWriter::AlarmTriggered, this)) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Alarm IO setup.
  wiringPiSetup();
  pinMode(kAlarmGPIOPin, OUTPUT);
#endif
}

void ActuatorsWriter::WriterThread() {
  while (::ros::ok()) {
    ROS_DEBUG_STREAM_THROTTLE(1, "actuators_writer wrote 1 second of data to actuators @ " << kWriterPhasedLoopFrequency << "hz");
    
    // Write out the alarm signal.
#ifdef UAS_AT_UCLA_DEPLOYMENT
    digitalWrite(kAlarmGPIOPin, alarm_.ShouldAlarm() ? HIGH : LOW);
#endif

    // Wait until next iteration of loop.
    writer_phased_loop_.sleep();
  }
}

void ActuatorsWriter::AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence) {
  ROS_INFO_STREAM("Alarm triggered.");

  // Override any current alarms.
  alarm_.ClearAlerts();

  // Iterate through all on-off pairs and add them to the alarm sequence.
  for(int i = 0;i < alarm_sequence.on_off_cycles_size() - alarm_sequence.on_off_cycles_size() % 2;i += 2) {
    alarm_.AddAlert({alarm_sequence.on_off_cycles(i), alarm_sequence.on_off_cycles(i + 1)});
  }
}

} // namespace actuators_writer
} // namespace io
} // namespace controls
} // namespace src
