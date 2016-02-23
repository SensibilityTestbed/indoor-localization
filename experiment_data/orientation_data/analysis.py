import numpy
import matplotlib.pyplot as plt


# Parses txt file into lists of data
def parse_sensor_data(filename):
  timestamps = []
  turntimes = []
  gyrolist = []
  xmags = []
  ymags = []
  # approximate times phone began to turn
  preturns = [94, 234, 354, 489, 594, 719, 823, 936, 1043, 1159, 1264, 1388]
  count = 0  # to check if the entry is a preturn
  with open(filename, 'r') as f:
    for line in f:
      # data is separated by tabs
      entry = line.split('\t')
      # remove leftover empty strings
      entry = list(filter(lambda element: element != '', entry))
      # It is ground truth data?
      if len(entry) == 3:
        # Just need the timestamp for turning
        turntimes.append(float(entry[0]))
      # Or is it sensor data?
      elif len(entry) == 10:
        # Was this data a "turning" point?
        if count in preturns: 
          # Save the timestamp for lower bound of turning interval
          turntimes.append(float(entry[0]))
        # Save relevant data
        timestamps.append(float(entry[0]))
        gyrolist.append(float(entry[6]))
        xmags.append(float(entry[7]))
        ymags.append(float(entry[8]))  
      count += 1

  return (timestamps, turntimes, gyrolist, xmags, ymags)



# Removes from data time intervals between turns 
def remove_turns(data, timestamps, turntimes):
  newdata = []
  cnt = 0
  # upper bound of turning interval
  upper = 1
  # we can quit the loop early if turntimes is odd
  even = len(turntimes) % 2 == 0
  while upper < len(turntimes):
    # keep the data in this interval
    while timestamps[cnt] < turntimes[upper]:
      newdata.append(data[cnt])
      cnt += 1

    upper += 1
    # check next boundaries are valid
    if (even and upper < len(turntimes)) or (not even and upper < (len(turntimes) - 1)):
      # skip next interval
      while timestamps[cnt] < turntimes[upper]:
        cnt += 1
    upper += 1

  return newdata


# Integrates gyroscope readings to obtain the headings (in degrees).
# Headings are relative to the first reading, which we take to be 0 deg. 
def gyro_headings(gyrolist, timestamps):
  headings = []
  prevheading = 0.0
  prevtime = timestamps[0]
  for index in range(len(gyrolist)):
    # calculate the time step
    dt = timestamps[index] - prevtime
    prevtime = timestamps[index]
    # integrate gyro and convert to degrees
    headings.append((gyrolist[index] * dt * 180 / numpy.pi + prevheading) % 360)
    prevheading = headings[index]
  return headings

# Calculates headings (in degrees) from magnetometer readings in the x-y plane.
# Headings are relative to magnetic North.
def mag_headings(xmags, ymags):  
  headings = []
  for index in range(len(xmags)):
    # The following algorithm assumes the sensor is level.
    # See magnetometer data sheet at http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
    if ymags[index] == 0:
      if xmags[index] < 0:
        headings.append(180.0)
      else:
        headings.append(0.0)
    else:
      head = numpy.arctan2(xmags[index], ymags[index]) * 180 / numpy.pi 
      if ymags[index] > 0:
        headings.append((90 - head) % 360)
      else: 
        headings.append((270 - head) % 360)
  return headings

# Calculates the mean and standard deviation of the difference
# between the actual heading and the sensor's heading.
def heading_diff_stats(headings, timestamps, turntimes):
  diff = []
  # If the sensor is a magnetometer,
  # it has an initial angle that should
  # be accounted for.
  offset = headings[0]
  turn = 1  # 0th turn is actually initial angle
  for index in range(len(timestamps)):
    # Have we made our last turn or are we before the next turn? 
    if timestamps[index] < turntimes[turn]:
      # turns add 90 deg
      actual = 90 * (turn-1) % 360
      diff.append(actual  - (headings[index] - offset))
    else:
      turn += 1
  print diff
  return (numpy.mean(diff), numpy.std(diff))


def sensor_diff_stats(gyroheadings, magheadings):
  diff = []
  # Magnetometer has an initial angle,
  # so we should subtract it to match gyro.
  offset = magheadings[0]
  for index in range(len(gyroheadings)):
    diff.append(gyroheadings[index] - (magheadings[index] - offset))
 
  return (numpy.mean(diff), numpy.std(diff))




if __name__ == "__main__":
  timestamps, turntimes, gyro, xmags, ymags = parse_sensor_data('data.txt')

  #print 'Filtered data:\n'
  window = [1.0/9] * 9 # half second window 
  smooth_gyro = numpy.convolve(gyro, window, mode='same')
  #print 'gyro: ' + str(smooth_gyro) + '\n' 
  smooth_xmags = numpy.convolve(xmags, window, mode='same')
  #print 'xmags: ' + str(smooth_xmags) + '\n'
  smooth_ymags = numpy.convolve(ymags, window, mode='same')
  #print 'ymags: ' + str(smooth_ymags) + '\n\n\n'


  plt.figure(num=1, figsize=(20, 8), dpi=80)
  plt.subplot(1, 1, 1)
  plt.plot(timestamps, gyro, color='blue', linewidth=1.0, linestyle='-')
  plt.plot(timestamps, smooth_gyro, color='red', linewidth=1.0, linestyle='-')
  plt.xlim(timestamps[0], timestamps[len(timestamps)-1])
  plt.show()
 
  plt.figure(num=2, figsize=(20, 8), dpi=80)
  plt.subplot(2, 1, 1)
  plt.plot(timestamps, xmags, color='blue', linewidth=1.0, linestyle='-')
  plt.plot(timestamps, smooth_xmags, color='red', linewidth=1.0, linestyle='-')
  plt.xlim(timestamps[0], timestamps[len(timestamps)-1])
  plt.subplot(2, 1, 2)
  plt.plot(timestamps, ymags, color='blue', linewidth=1.0, linestyle='-')
  plt.plot(timestamps, smooth_ymags, color='red', linewidth=1.0, linestyle='-')
  plt.xlim(timestamps[0], timestamps[len(timestamps)-1])
  plt.show()



  print 'Headings:\n' 
  gyro_head = gyro_headings(gyro, timestamps)
  print 'gyro: ' + str(gyro_head) + '\n'
  mag_head = mag_headings(xmags, ymags)
  print 'mag: ' + str(mag_head) + '\n\n'
  
  plt.figure(num=3, figsize=(20, 8), dpi=80)
  plt.subplot(1, 1, 1)
  plt.plot(timestamps, gyro_head, color='blue', linewidth=1.0, linestyle='-')
  plt.plot(timestamps, mag_head, color='red', linewidth=1.0, linestyle='-')
  plt.xlim(timestamps[0], timestamps[len(timestamps)-1])
  plt.show()
 

  print 'Filtered Headings:\n'
  smooth_gyro_head = gyro_headings(smooth_gyro, timestamps)
  print 'gyro: ' + str(smooth_gyro_head) + '\n'
  smooth_mag_head = mag_headings(smooth_xmags, smooth_ymags)
  print 'mag: ' + str(smooth_mag_head) + '\n\n\n'

  gyro_head = remove_turns(gyro_head, timestamps, turntimes)
  mag_head = remove_turns(mag_head, timestamps, turntimes)
  smooth_gyro_head = remove_turns(smooth_gyro_head, timestamps, turntimes)
  smooth_mag_head = remove_turns(smooth_mag_head, timestamps, turntimes)
  timestamps = remove_turns(timestamps, timestamps, turntimes)

  # remove lower bounds of turn times, so we know only when the angle is done changing
  turntimes = [turntimes[time] for time in range(len(turntimes)) if time % 2 == 0]

  print 'Mean Difference from Actual Angle:\n'
  gyro_diff_mean, gyro_diff_std = heading_diff_stats(gyro_head, timestamps, turntimes)
  print 'gyro: ' + str(gyro_diff_mean) + '\n'
  mag_diff_mean, mag_diff_std = heading_diff_stats(mag_head, timestamps, turntimes)
  print 'mag: ' + str(mag_diff_mean) + '\n\n'

  print 'Standard deviation of Difference from Actual Angle:\n'
  print 'gyro: ' + str(gyro_diff_std) + '\n'
  print 'mag: ' + str(mag_diff_std) + '\n\n'
  
  print 'Mean Filtered Difference from Actual Angle:\n'
  smooth_gyro_diff_mean, smooth_gyro_diff_std = heading_diff_stats(smooth_gyro_head, timestamps, turntimes)
  print 'gyro: ' + str(smooth_gyro_diff_mean) + '\n'
  smooth_mag_diff_mean, smooth_mag_diff_std = heading_diff_stats(smooth_mag_head, timestamps, turntimes)
  print 'mag: ' + str(smooth_mag_diff_mean) + '\n\n'

  print 'Standard deviation of Filtered Difference from Actual Angle:\n'
  print 'gyro: ' + str(smooth_gyro_diff_std) + '\n'
  print 'mag: ' + str(smooth_mag_diff_std) + '\n\n\n'

  print 'Mean Differnece Between Sensors:\n'
  gyro_mag_mean, gyro_mag_std = sensor_diff_stats(gyro_head, mag_head)
  print 'unfiltered: ' + str(gyro_mag_mean) + '\n'
  smooth_gyro_mag_mean, smooth_gyro_mag_std = sensor_diff_stats(smooth_gyro_head, smooth_mag_head)
  print 'filtered: ' + str(smooth_gyro_mag_mean) + '\n\n'
  
  print 'Standard deviation of Sensor Difference:\n'
  print 'unfiltered: ' + str(gyro_mag_std) + '\n'
  print 'filtered: ' + str(smooth_gyro_mag_std) + '\n\n\n'
