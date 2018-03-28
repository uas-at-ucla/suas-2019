#ifndef SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
#define SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_

#include <mutex>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <vector>

namespace lib {

class Semaphore {
 public:
  Semaphore(unsigned long count);
  void Notify();
  void Wait();
  bool TryWait();

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned long count_;
};

class MissionCommand {
 public:
  MissionCommand();

  enum Type { GOTO, BOMB_DROP, DO_NOTHING };

  virtual Type type() = 0;
};

class MissionCommandGoto : public MissionCommand {
 public:
  MissionCommandGoto(double latitude, double longitude, double altitude);
  MissionCommandGoto(MissionCommandGoto* cmd);

  Type type() override { return GOTO; }

  double latitude() { return latitude_; }
  double longitude() { return longitude_; }
  double altitude() { return altitude_; }

 private:
  double latitude_;
  double longitude_;
  double altitude_;
};

class MissionCommandBombDrop : public MissionCommand {
 public:
  MissionCommandBombDrop();
  MissionCommandBombDrop(MissionCommandBombDrop* cmd);

  Type type() override { return BOMB_DROP; }
};

class MissionCommandDoNothing : public MissionCommand {
 public:
  MissionCommandDoNothing();
  MissionCommandDoNothing(MissionCommandDoNothing* cmd);

  Type type() override { return DO_NOTHING; }
};

class MissionManager {
 public:
  MissionManager();
  void AddCommands(
      ::std::vector<::std::shared_ptr<MissionCommand>> new_commands);
  void ClearCommands();
  void PopCommand();
  size_t NumberOfCommands();

  ::std::shared_ptr<MissionCommand> GetCurrentCommand();

 private:
  ::std::vector<::std::shared_ptr<MissionCommand>> commands_;

  Semaphore semaphore_;
  size_t command_pointer_;
};

}  // namespace lib

#endif  // SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
