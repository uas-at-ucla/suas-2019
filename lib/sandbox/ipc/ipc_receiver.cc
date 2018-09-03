#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>

int main(void) {
  int mq; /* the queue handler */
  /* create a key for the queue */
  key_t k = ftok("/var", 'c');
  if (k != -1) { /* did we succeed? */
    /* get the queue (create if it doesn't exist) */
    mq = msgget(k, 0644 | IPC_CREAT);
    if (mq != -1) {       /* did we succeed */
      char b[10000] = ""; /* a message buffer */
      /* keep waiting on messages */
      while (1) { /* keep going */
        /* attempt to receive a message */
        if (msgrcv(mq, &b, sizeof(b), 0, IPC_NOWAIT) != -1) {
          /* was it the `stop' instruction? */
          if (strcmp(b, "stop") == 0)
            break;
          printf("%s\n", b); /* print the message */
        } else {
          perror("msgrcv()");
          continue;
          break;
        }
      }
    } else {
      perror("msgget()");
      return 2;
    }
  } else {
    perror("ftok()");
    return 1;
  }

  return 0;
}
