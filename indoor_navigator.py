queue = 
q_lock = createlock()
location =
loc_lock = createlock()
start = False


class IndoorNavigator:
  def __init__():
    self.pedometer = Pedometer()
    self.compass = OrientationFilter()
    self.queue = PriorityQueue()
    self.coord = [[0.0, 0.0]]
    self.qlock = createlock()
    self.coordlock = creatlock() 
    createthread(_process_data) 



def navigate(sensordata):
  qlock.acquire(True)
  queue.insert(sensordata)
  qlock.release()



def get_coordinates():
  coordlock.acquire(True)
  coord = self.coord[0]
  coordlock.release()
  return coord



def _process_data():
  while True:
    qlock.acquire(True):
    if queue.is_empty():
      qlock.release()
      continue
    data = queue.deleteMinimum()
    qlock.release()

    q = self.compass.get_orientation(data)
    if self.pedometer.detect_step(0, data, data['time']):
      coordlock.acquire(True)
      coord = matrix_add(coord, [compass.get_heading()])
      coordlock.release()
