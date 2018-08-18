#pragma once

#include <string>
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/events/raw-event-loop.h"

namespace aos {

// Fetches the newest message from a queue.
template <typename T>
class Fetcher {
 public:
  Fetcher() {}
  // Fetches the most recent message. Returns whether it fetched a new message.
  bool Fetch() { return fetcher_->Fetch(); }

  const T *get() const {
    return reinterpret_cast<const T *>(fetcher_->most_recent());
  }
  const T &operator*() const { return *get(); }
  const T *operator->() const { return get(); }

 private:
  friend class EventLoop;
  Fetcher(std::unique_ptr<RawFetcher> fetcher) : fetcher_(std::move(fetcher)) {}
  std::unique_ptr<RawFetcher> fetcher_;
};

// Sends messages to a queue.
template <typename T>
class Sender {
 public:
  Sender() {}

  // Represents a single message about to be sent to the queue.
  // The lifecycle goes:
  //
  // Message msg = sender.MakeMessage();
  // Populate(msg.get());
  // msg.Send();
  //
  // Or:
  //
  // Message msg = sender.MakeMessage();
  // PopulateOrNot(msg.get());
  class Message {
   public:
    Message(RawSender *sender)
        : msg_(reinterpret_cast<T *>(sender->GetContext()), *sender) {
      msg_->Zero();
    }

    T *get() { return msg_.get(); }
    const T *get() const { return msg_.get(); }
    T &operator*() { return *get(); }
    T *operator->() { return get(); }
    const T &operator*() const { return *get(); }
    const T *operator->() const { return get(); }

    // Sends the message to the queue. Should only be called once.
    void Send() {
      RawSender *sender = &msg_.get_deleter();
      msg_->SetTimeToNow();
      sender->Send(reinterpret_cast<RawSender::SendContext *>(msg_.release()));
    }

   private:
    std::unique_ptr<T, RawSender &> msg_;
  };

  // Constructs an above message.
  Message MakeMessage();

 private:
  friend class EventLoop;
  Sender(std::unique_ptr<RawSender> sender) : sender_(std::move(sender)) {}
  std::unique_ptr<RawSender> sender_;
};

// TODO(parker): Consider making EventLoop wrap a RawEventLoop rather than
// inheriting.
class EventLoop : public RawEventLoop {
 public:
  virtual ~EventLoop() {}

  // Current time.
  virtual monotonic_clock::time_point monotonic_now() = 0;

  // Makes a class that will always fetch the most recent value
  // sent to path.
  template <typename T>
  Fetcher<T> MakeFetcher(const std::string &path) {
    return Fetcher<T>(MakeRawFetcher(path, QueueTypeInfo::Get<T>()));
  }

  // Makes class that allows constructing and sending messages to
  // address path.
  template <typename T>
  Sender<T> MakeSender(const std::string &path) {
    return Sender<T>(MakeRawSender(path, QueueTypeInfo::Get<T>()));
  }

  // Watch is a functor that have a call signature like so:
  // void Event(const MessageType& type);
  //
  // This will watch messages sent to path.
  // Note that T needs to match both send and recv side.
  template <typename Watch>
  void MakeWatcher(const std::string &path, Watch &&w);

  // The passed in function will be called when the event loop starts.
  // Use this to run code once the thread goes into "real-time-mode",
  virtual void OnRun(std::function<void()>) = 0;

  // Starts receiving events.
  virtual void Run() = 0;

  // TODO(austin): Sort out how to switch to realtime on run.
  // virtual void RunRealtime() = 0;

  // Stops receiving events
  virtual void Exit() = 0;
};

}  // namespace aos

#include "aos/events/event-loop-tmpl.h"

