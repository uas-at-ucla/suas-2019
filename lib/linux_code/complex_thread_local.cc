#include "lib/linux_code/complex_thread_local.h"

#include <pthread.h>

#include "lib/util/die/die.h"
#include "lib/util/once/once.h"

#define SIMPLE_CHECK(call)              \
  do {                                  \
    const int value = call;             \
    if (value != 0) {                   \
      PRDie(value, "%s failed", #call); \
    }                                   \
  } while (false)

namespace lib {
namespace util {
namespace {

void ExecuteDestructorList(void *v) {
  for (const ComplexThreadLocalDestructor *c =
           static_cast<ComplexThreadLocalDestructor *>(v);
       c != nullptr; c = c->next) {
    c->function(c->param);
  }
}

pthread_key_t *CreateKey() {
  static pthread_key_t r;
  SIMPLE_CHECK(pthread_key_create(&r, ExecuteDestructorList));
  return &r;
}

::lib::util::Once<pthread_key_t> key_once(CreateKey);

} // namespace

void ComplexThreadLocalDestructor::Add() {
  static_assert(
      ::std::is_pod<ComplexThreadLocalDestructor>::value,
      "ComplexThreadLocalDestructor might not be safe to pass through void*");
  pthread_key_t *const key = key_once.Get();

  next = static_cast<ComplexThreadLocalDestructor *>(pthread_getspecific(*key));
  SIMPLE_CHECK(pthread_setspecific(*key, this));
}

void ComplexThreadLocalDestructor::Remove() {
  pthread_key_t *const key = key_once.Get();

  ComplexThreadLocalDestructor *previous = nullptr;
  for (ComplexThreadLocalDestructor *c =
           static_cast<ComplexThreadLocalDestructor *>(
               pthread_getspecific(*key));
       c != nullptr; c = c->next) {
    if (c == this) {
      // If it's the first one.
      if (previous == nullptr) {
        SIMPLE_CHECK(pthread_setspecific(*key, next));
      } else {
        previous->next = next;
      }
      return;
    }
    previous = c;
  }
  ::lib::util::die::Die("%p is not in the destructor list\n", this);
}

}  // namespace util
}  // namespace lib
