import threading
import serial
import argparse
import graphyte
import pickle
import socket
import struct
import csv
import time
# recommend to install python-prctl, so the threads are properly named
# but it's not mandatory
try:
  import prctl
  prctl_exists = True
except:
  prctl_exists = False
from pprint import pprint


def average_rows(batch_for_avg):
  avg_row = [0.0] * 24
  first_row = batch_for_avg[0]

  # just copy timestamp and labels
  for i in [0, 1, 5, 9, 13, 17, 22]:
    avg_row[i] = first_row[i]

  for j in [2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20, 21, 23]:
    for i in range(len(batch_for_avg)):
      avg_row[j] += float(batch_for_avg[i][j])
    avg_row[j] /= len(batch_for_avg)

  return(avg_row)


def strings_to_tuples(messages, gpath):
  # take in a list of strings; one string = one line of full sensor data with timestamp
  # return a list of tuples with sensor data ready for pickle
  # guarantee unique timestamps for each metric

  output_batch = []

  ts_old = ''
  batch_for_avg = []

  for row in csv.reader(messages):
    ts = row[0]
    # are we still within the same second?
    if ts == ts_old:
      # we are within the same second-long sequence
      # just keep absorbing rows
      batch_for_avg.append(row)
      continue
    else:
      # we've reached the cusp between seconds
      if batch_for_avg:
        # we've collected at least one row for the previous second
        # do the average and continue processing
        avg_row = average_rows(batch_for_avg)
        batch_for_avg = [row]
      else:
        # no previous data for this second
        # perhaps the thread just got started
        # add row to the pile and keep collecting rows
        batch_for_avg = [row]
        continue

    ts_old = ts
    air_t = (gpath + '.air.temperature', (ts, avg_row[2]))
    air_h = (gpath + '.air.humidity', (ts, avg_row[3]))
    air_p = (gpath + '.air.pressure', (ts, avg_row[4]))
    acc_x = (gpath + '.acceleration.x', (ts, avg_row[6]))
    acc_y = (gpath + '.acceleration.y', (ts, avg_row[7]))
    acc_z = (gpath + '.acceleration.z', (ts, avg_row[8]))
    gyro_x = (gpath + '.gyroscope.x', (ts, avg_row[10]))
    gyro_y = (gpath + '.gyroscope.y', (ts, avg_row[11]))
    gyro_z = (gpath + '.gyroscope.z', (ts, avg_row[12]))
    magn_x = (gpath + '.magnetic.x', (ts, avg_row[14]))
    magn_y = (gpath + '.magnetic.y', (ts, avg_row[15]))
    magn_z = (gpath + '.magnetic.z', (ts, avg_row[16]))
    light_r = (gpath + '.light.red', (ts, avg_row[18]))
    light_g = (gpath + '.light.green', (ts, avg_row[19]))
    light_b = (gpath + '.light.blue', (ts, avg_row[20]))
    light_w = (gpath + '.light.white', (ts, avg_row[21]))
    noise = (gpath + '.noise', (ts, avg_row[23]))
    datapoint = [air_t, air_h, air_p, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,
                 magn_x, magn_y, magn_z, light_r, light_g, light_b, light_w, noise]
    output_batch += datapoint

  return output_batch


def reader_main(lock, args):
  # keep this function simple
  global msgs
  global prctl_exists

  if prctl_exists:
    prctl.set_name(threading.currentThread().name)

  now_old = ''
  msg_in = []

  ser = serial.Serial()
  ser.baudrate = args.speed
  ser.port = args.port
  ser.open()

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
  global prctl_exists

  if prctl_exists:
    prctl.set_name(threading.currentThread().name)

  msgs_out = []

  while True:
    time.sleep(args.gwait)
    if len(msgs) != 0:
      #copy and empty out the inter-thread buffer
      lock.acquire()
      msgs_out = msgs
      msgs = []
      lock.release()

      # Graphite pickle protocol
      # rewrite messages as list of tuples
      graph_slice = strings_to_tuples(msgs_out, args.gpath)
      print()
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

  # Using a list, not a thread-safe queue. So use a lock then.
  lock = threading.Lock()

  reader = threading.Thread(target = reader_main, name = 'reader', args = (lock, args), daemon = True)
  writer = threading.Thread(target = writer_main, name = 'writer', args = (lock, args), daemon = True)

  reader.start()
  writer.start()

  reader.join()
  writer.join()
