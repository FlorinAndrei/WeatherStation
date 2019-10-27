import threading
import serial
import argparse
import graphyte
import time


def reader_main(lock, ser):
  global msgs

  while True:
    msg_line = ser.readline().decode('UTF-8').rstrip()
    now = str(int(time.time()))
    msg_line = now + ',' + msg_line
    lock.acquire()
    msgs.append(msg_line)
    lock.release()


def writer_main(lock):
  global msgs
  msgs_write = []
  print("Ready:")

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
  parser = argparse.ArgumentParser(description="data relay and message queue")
  parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='serial port connected to Arduino')
  parser.add_argument('--speed', type=int, default=115200, help='serial port speed')
  args = parser.parse_args()

  # This is not a queue. It is truncated all at once by the consumer.
  # No point in using a queue then. So it needs locking.
  msgs = []

  ser_port = args.port
  ser_speed = args.speed

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
