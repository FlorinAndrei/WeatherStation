import threading
import serial
import time


# This is not a queue. It is truncated all at once by the consumer.
# No point in using a queue then. So it needs locking.
msgs = []

ser_port = '/dev/ttyACM0'
ser_speed = 9600


def reader_main(lock, ser):
  global msgs

  while True:
    msg_line = ser.readline().decode('UTF-8').rstrip()
    lock.acquire()
    msgs.append(msg_line)
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

  ser = serial.Serial()
  ser.baudrate = ser_speed
  ser.port = ser_port
  ser.open()

  reader = threading.Thread(target = reader_main, args = (lock, ser), daemon = True)
  writer = threading.Thread(target = writer_main, args = (lock,), daemon = True)

  reader.start()
  writer.start()

  reader.join()
  writer.join()
