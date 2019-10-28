import threading
import serial
import argparse
import graphyte
import pickle
import socket
import struct
import csv
import time
from pprint import pprint


def text_to_pickle(messages, gpath):
  # take in a list of strings; one string = one line of full sensor data with timestamp
  # return a list of tuples with sensor data ready for pickle
  # guarantee unique timestamps for each metric

  output_batch = []

  ts_old = ''
  for row in csv.reader(messages):
    ts = row[0]
    if ts == ts_old:
      # skip duplicate timestamps for now
      # TODO: average them instead
      continue
    ts_old = ts
    air_t = (gpath + '.air.temperature', (ts, row[2]))
    air_h = (gpath + '.air.humidity', (ts, row[3]))
    air_p = (gpath + '.air.pressure', (ts, row[4]))
    acc_x = (gpath + '.acceleration.x', (ts, row[6]))
    acc_y = (gpath + '.acceleration.y', (ts, row[7]))
    acc_z = (gpath + '.acceleration.z', (ts, row[8]))
    gyro_x = (gpath + '.gyroscope.x', (ts, row[10]))
    gyro_y = (gpath + '.gyroscope.y', (ts, row[11]))
    gyro_z = (gpath + '.gyroscope.z', (ts, row[12]))
    magn_x = (gpath + '.magnetic.x', (ts, row[14]))
    magn_y = (gpath + '.magnetic.y', (ts, row[15]))
    magn_z = (gpath + '.magnetic.z', (ts, row[16]))
    light_r = (gpath + '.light.red', (ts, row[18]))
    light_g = (gpath + '.light.green', (ts, row[19]))
    light_b = (gpath + '.light.blue', (ts, row[20]))
    light_w = (gpath + '.light.white', (ts, row[21]))
    noise = (gpath + '.noise', (ts, row[23]))
    datapoint = [air_t, air_h, air_p, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,
                 magn_x, magn_y, magn_z, light_r, light_g, light_b, light_w, noise]
    output_batch += datapoint

  return output_batch


def reader_main(lock, ser):
  # keep this function as simple as possible
  # to avoid losing data from the Arduino
  global msgs
  now_old = ''
  msg_in = []

  while True:
    msg_line = ser.readline().decode('UTF-8').rstrip()
    now = str(int(time.time()))
    msg_line = now + ',' + msg_line
    if now != now_old:
      # a new second has started; flush the local buffer
      # we only do this on the cusp between seconds
      # so the writer thread only receives complete seconds
      lock.acquire()
      msgs += msg_in
      lock.release()
      msg_in = []
      now_old = now
    msg_in.append(msg_line)


def writer_main(lock, args):
  global msgs
  msgs_out = []

  while True:
    if len(msgs) != 0:
      #copy and empty out the inter-thread buffer
      lock.acquire()
      msgs_out = msgs
      msgs = []
      lock.release()

      # Graphite pickle protocol
      # rewrite messages as list of tuples
      graph_slice = text_to_pickle(msgs_out, args.gpath)
      pprint(graph_slice)
      # prepare the Graphite data
      payload = pickle.dumps(graph_slice, protocol=2)
      header = struct.pack("!L", len(payload))

      try:
        # We don't maintain a permanent connection to Graphite.
        # Only connect occasionally (default: 10 sec pause) and send data.
        # Breaking connection is fine if the pause is long enough.
        # If Graphite is not available, cache data internally (TBD).
        sock = socket.socket()
        sock.connect((args.gserver, args.gport))
        sock.sendall(header)
        sock.sendall(payload)
        sock.close()
      except:
        # this will lose data if Graphite is not available
        # TODO: implement internal buffer limited by usable RAM
        continue

    time.sleep(args.gwait)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="data relay and message queue")
  parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='serial port connected to Arduino')
  parser.add_argument('--speed', type=int, default=115200, help='serial port speed')
  parser.add_argument('--gserver', type=str, required=True, help='Graphite server host / IP')
  parser.add_argument('--gport', type=int, default=2004, help='Graphite server port')
  parser.add_argument('--gwait', type=int, default=10, help='Pause duration between Graphite data dumps')
  parser.add_argument('--gpath', type=str, default='weather', help='Graphite path prefix')
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
  writer = threading.Thread(target = writer_main, args = (lock, args), daemon = True)

  reader.start()
  writer.start()

  reader.join()
  writer.join()
