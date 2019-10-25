import threading
import time


# This is not a queue. It is truncated all at once by the consumer.
# No point in using a queue then. So it needs locking.
msgs = []


def reader_main(lock):
  global msgs

  while True:
    msg_read = input('input: ')
    lock.acquire()
    msgs.append(msg_read)
    lock.release()


def writer_main(lock):
  global msgs
  msgs_write = []

  while True:
    if len(msgs) != 0:
      lock.acquire()
      msgs_write += msgs
      msgs = []
      lock.release()
      print("OUTPUT:")
      for m in msgs_write:
        print(m)
      msgs_write = []
    time.sleep(10)


if __name__ == "__main__":
  # Using a list, not a thread-safe queue. So use a lock then.
  lock = threading.Lock()

  reader = threading.Thread(target = reader_main, args = (lock,), daemon = True)
  writer = threading.Thread(target = writer_main, args = (lock,), daemon = True)

  reader.start()
  writer.start()

  reader.join()
  writer.join()
