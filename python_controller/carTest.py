
import csv
import math
import os.path
import sys
import numpy as np

import vrep

class VRepAPIError(Exception):
  pass

class VRepInterfaceError(Exception):
  pass

class VRepInterfaceFactory(object):
  def __init__(self, connection_addr, connection_port, timeout, thread_period):
    self.connection_addr = connection_addr
    self.connection_port = connection_port
    self.timeout = timeout
    self.thread_period = thread_period

  def __enter__(self):
    """Opens a V-REP API connection or dies trying.
    """
    client_id = vrep.simxStart(self.connection_addr, self.connection_port,
                               True, True, self.timeout, self.thread_period)
    if client_id == -1:
      raise VRepAPIError("Connection failed")

    self.vrep_interface = VRepInterface(client_id)
    return self.vrep_interface

  def __exit__(self, type, value, traceback):
    self.vrep_interface.close()

class VRepInterface(object):
  """A more Pythonic V-REP interface, storing state data (like client ID) in an
  object and using exceptions to avoid checking returns everywhere.
  """
  def __init__(self, client_id):
    assert client_id != -1
    self.client_id = client_id
    pass

  @staticmethod
  def open(connection_addr='127.0.0.1', connection_port=19997,
           timeout=500, thread_period=5):
    return VRepInterfaceFactory(connection_addr, connection_port, timeout,
                                thread_period)

  def __getattr__(self, name):
    """Pass-through for V-Rep API functions, removing the need to pass in a
    clientID everywhere and automatically checking the return code. Should work
    for the vast majority of V-Rep API functions...
    """
    if not hasattr(vrep, name):
      raise VRepInterfaceError("V-Rep API has no function '%s'" % name)

    vrep_fn = getattr(vrep, name)

    def inner(*args, **kwargs):
      args = (self.client_id, ) + args
      result_packed = vrep_fn(*args, **kwargs)

      if isinstance(result_packed, tuple):
        result_code = result_packed[0]
        result = result_packed[1:]
        if len(result) == 1:
          result = result[0]
      elif isinstance(result_packed, int):
        result_code = result_packed
        result = None
      else:
        raise VRepInterfaceError("Unexpected result format from API call '%s' (args=%s, kwargs=%s), got %s"
                                 % (name, args, kwargs, result_packed))

      if (result_code != vrep.simx_return_ok
          and result_code != vrep.simx_return_novalue_flag):
        # TODO: what circumstances are the second not an error in?!
        raise VRepAPIError("API call '%s' (args=%s, kwargs=%s), failed (got %i)"
                           % (name, args, kwargs, result_code))
      return result

    return inner

  def close(self):
    """Cleans up the V-Rep API connection. Call this when done.
    """
    if self.client_id != -1:
      vrep.simxFinish(self.client_id)

  #
  # Helper functions to abstract away lower-level functionality
  #
  def get_bounding_box(self, name):
    handle = self.simxGetObjectHandle(name, vrep.simx_opmode_oneshot_wait)
    xmin = self.simxGetObjectFloatParameter(handle, 15,
                                            vrep.simx_opmode_oneshot_wait)
    ymin = self.simxGetObjectFloatParameter(handle, 16,
                                            vrep.simx_opmode_oneshot_wait)
    zmin = self.simxGetObjectFloatParameter(handle, 17,
                                            vrep.simx_opmode_oneshot_wait)
    xmax = self.simxGetObjectFloatParameter(handle, 18,
                                            vrep.simx_opmode_oneshot_wait)
    ymax = self.simxGetObjectFloatParameter(handle, 19,
                                            vrep.simx_opmode_oneshot_wait)
    zmax = self.simxGetObjectFloatParameter(handle, 20,
                                            vrep.simx_opmode_oneshot_wait)
    return ((xmin, xmax), (ymin, ymax), (zmin, zmax))

  def get_bounding_size(self, name):
    bbox = self.get_bounding_box(name)
    return (bbox[0][1]-bbox[0][0], bbox[1][1]-bbox[1][0], bbox[2][1]-bbox[2][0])

class Car(object):
  """Abstraction object for the car, providing intuitive functions for getting
  car state and setting outputs.
  """
  def __init__(self, vrep_interface, name='AckermannSteeringCar'):
    self.vr = vrep_interface
    self.car_handle = self.vr.simxGetObjectHandle(name,
                                                  vrep.simx_opmode_oneshot_wait)
    self.boom_handle = self.vr.simxGetObjectHandle('BoomSensor',
                                                   vrep.simx_opmode_oneshot_wait)
    self.camera_handle = []
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera0',
                                                          vrep.simx_opmode_oneshot_wait))
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera1',
                                                          vrep.simx_opmode_oneshot_wait))

    # Open these variables in streaming mode for high efficiency access.
    self.vr.simxGetObjectPosition(self.car_handle, -1, vrep.simx_opmode_streaming)
    self.vr.simxGetObjectVelocity(self.car_handle, vrep.simx_opmode_streaming)

    self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_streaming)
    self.vr.simxGetFloatSignal('steerAngle', vrep.simx_opmode_streaming)

    for handle in self.camera_handle:
      self.vr.simxGetVisionSensorImage(handle, 1, vrep.simx_opmode_streaming)

    # Get the wheel diameter so we can set velocity in physical units.
    # Let's assume all the wheels are the same.
    # TODO: check this assumption?
    wheel_dia = self.vr.get_bounding_size('RearLeftWheel_respondable')[0]
    # multiply linear velocity by this to get wheel radians/s
    self.speed_factor = 2 / wheel_dia

    # Default parameters
    self.steering_limit = 30
  #
  # Some helper functions to get/set important car data/state
  #
  def get_position(self):
    """Returns the car's absolute position as a 3-tuple of (x, y, z), in meters.
    """
    # Specify -1 to retrieve the absolute position.
    return self.vr.simxGetObjectPosition(self.car_handle, -1,
                                         vrep.simx_opmode_buffer)

  def get_velocity(self):
    """Returns the car's linear velocity as a 3-tuple of (x, y, z), in m/s.
    """
    return self.vr.simxGetObjectVelocity(self.car_handle,
                                         vrep.simx_opmode_buffer)[0]

  def get_steering_angle(self):
    """Returns the car's steering angle in degrees.
    """
    return math.degrees(self.vr.simxGetFloatSignal('steerAngle',
                                                   vrep.simx_opmode_buffer))

  def get_lateral_error(self):
    """Returns the lateral error (distance from sensor to line) in meters.
    """
    return self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_buffer)

  def get_line_camera_image(self, camera_index):
    """Returns the line sensor image as an array of pixels, where each pixel is
    between [0, 255], with 255 being brightest.
    Likely non-physical (graphical) units of linear perceived pixel intensity.
    """
    # Magical '1' argument specifies to return the data as greyscale.
    handle = self.camera_handle[camera_index]
    _, image = self.vr.simxGetVisionSensorImage(handle, 1,
                                                vrep.simx_opmode_buffer)
    for i, intensity in enumerate(image):
      if intensity < 0:	# undo two's complement
        image[i] = 256 + intensity
    return image

  def set_speed(self, speed, blocking=False):
    """Sets the car's target speed in m/s. Subject to acceleration limiting in
    the simulator.
    """
    if blocking:
      op_mode = vrep.simx_opmode_oneshot_wait
    else:
      op_mode = vrep.simx_opmode_oneshot
    self.vr.simxSetFloatSignal('xSpeed', speed*self.speed_factor, op_mode)

  def set_steering(self, angle):
    """Sets the car's steering angle in degrees.
    Steering angle limiting happens here.
    Returns the actual commanded angle.
    """
    angle = min(angle, self.steering_limit)
    angle = max(-self.steering_limit, angle)
    self.vr.simxSetFloatSignal('steerAngle', angle*(math.pi/180.0),
                               vrep.simx_opmode_oneshot)
    return angle

  def set_boom_sensor_offset(self, boom_length):
    """Sets the car's boom sensor's offset (approximate distance from front of
    car, in meters).
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your boom sensor parameters.
    NOTE: this doesn't update the graphical boom stick.
    """
    self.vr.simxSetObjectPosition(self.boom_handle, vrep.sim_handle_parent,
                                  (0, 0, -(boom_length-0.35)),
                                  vrep.simx_opmode_oneshot_wait)

  def set_line_camera_parameters(self, camera_index, height=0.2,
                                 orientation=30, fov=90):
    """Sets the car's line camera parameters.
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your camera parameters.
    Args:
        height -- height of the camera, in meters.
        orientation -- downward angle of the camera, in degrees from horizontal.
        fov -- field of vision of the camera, in degrees.
    """
    handle = self.camera_handle[camera_index]
    self.vr.simxSetObjectPosition(handle, vrep.sim_handle_parent,
                                  (height, 0, 0),
                                  vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectOrientation(handle, vrep.sim_handle_parent,
                                     (0,
                                      -math.radians(orientation),
                                      math.radians(90)),
                                     vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectFloatParameter(handle, 1004, math.radians(fov),
                                        vrep.simx_opmode_oneshot_wait)

  def set_steering_limit(self, steering_limit):
    """Sets the car's steering limit in degrees from center.
    Attempts to steer past this angle (on either side) get clipped.
    """
    self.steering_limit = steering_limit

class SimulationAssignment():
  """Everything you need to implement for the assignment is here.
  You may """
  def __init__(self, vr, car):
    # You may initialize additional state variables here
    self.last_sim_time = vr.simxGetFloatSignal('simTime',
                                               vrep.simx_opmode_oneshot_wait)
    self.mid1 = 0
    self.last_err = None;

  def moving_average(self,values,window):
    weigths = np.repeat(1.0, window)/window
    #including valid will REQUIRE there to be enough datapoints.
    #for example, if you take out valid, it will start @ point one,
    #not having any prior points, so itll be 1+0+0 = 1 /3 = .3333
    smas = np.convolve(values, weigths, 'valid')
    return smas # as a numpy array

  def get_line_camera_error(self, image):
    """Returns the distance from the line, as seen by the line camera, in
    pixels. The actual physical distance (in meters) can be derived with some
    trig given the camera parameters.
    """
    #
    # ASSIGNMENT: You should implement your line tracking algorithm here.
    # The default code implements a very simple, very not-robust line detector.
    #
    # NOTE: unlike the actual camera, get_line_camera_image() returns pixel
    # intensity data in the range of [0, 255] and aren't physically based (i.e.
    # intensity in display intensity units, no integration time).
    #
    INTENSITY_THRESHOLD = 192
    MIDPOINT = 63
    weighted_sum = 0
    element_sum = 0
    max_value = 0
    min_value = 255
    max_index = 0
    min_index = 0
    # print image

    #TODO: our own line scanning algorithm
    # Low pass/ moving_average
    image = self.moving_average(image, 3)
    # print "LOW PASS: \n", image
    high_pass = list(image)
    # High pass
    for i in range(len(high_pass)):
      if (i == 0):
          continue
      else:
          high_pass[i] = image[i] - image[i-1]
    high_pass[0] = high_pass[1]


    # print "HIGH PASS: \n", high_pass

    for i in range(len(high_pass)):

        if high_pass[i] > max_value:
            max_index = i
            max_value = high_pass[i]

    for j in range(len(high_pass)):
        if j < max_index:
            continue
        elif high_pass[j] < min_value:
            min_index = j
            min_value = high_pass[j]


    result = (max_index + min_index) / 2.0

    if self.mid1 == 0:
        self.mid1 = result
        return (result - 63) * 0.3125 / 100

    change = self.mid1 - result
    if change < 0:
        change = -change
    WIDTH = 15

    if change < 20 and (min_index - max_index) < WIDTH:
        self.mid1 = result
        return (result - 63) * 0.3125 / 100
    # print result, '\n'
    return (self.mid1 - 63) * 0.3125 / 100

    # return result - midpoint
    # for i, intensity in enumerate(image):
    #   if intensity > INTENSITY_THRESHOLD:
    #     weighted_sum += i
    #     element_sum += 1
    # if element_sum == 0:
    #   return None
    # return weighted_sum / element_sum - midpoint

  def setup_car(self, vr, car):
    """Sets up the car's physical parameters.
    """
    #
    # ASSIGNMENT: You may want to tune these paraneters.
    #
    car.set_boom_sensor_offset(0.3)
    # In the scene, we provide two cameras.
    # Camera 0 is used by default in the control loop and is the "near" one.
    # Some teams have also used a "far" camera in the past (Camera 1).
    # You can use additional cameras, but will have to add then in the V-REP scene
    # and bind the handle in Car.__init__. The V-REP remote API doesn't provide a
    # way to instantiate additional vision sensors.
    # TODO: SET CAMERA CONFIG HERE
    car.set_line_camera_parameters(0, height=0.1, orientation= 60, fov= 90)
    car.set_line_camera_parameters(1, height=0.1, orientation= 60, fov= 90)
    # You should measure the steering sevo limit and set it here.
    # A more accurate approach would be to implement servo slew limiting.
    car.set_steering_limit(30)

  def control_loop(self, vr, car, csvfile=None):
    """Control iteration. This is called on a regular basis.
    Args:
        vr -- VRepInterface object, which is an abstraction on top of the VREP
              Python API. See the class docstring near the top of this file.
        car -- Car object, providing abstractions for the car (like steering and
               velocity control).
        csvfile -- Optional csv.DictWriter for logging data. None to disable.
    """
    # Waiting mode is used here to
    time = vr.simxGetFloatSignal('simTime', vrep.simx_opmode_oneshot_wait)
    dt = time - self.last_sim_time
    self.last_sim_time = time

    #
    # ASSIGNMENT: Tune / implement a better controller loop here.
    #

    # Here are two different sensors you can play with.
    # One provides an ideal shortest-distance-to-path (in meters), but isn't
    # path-following and may jump at crossings.
    # The other uses a more realistic line sensor (in pixels), which you can
    # plug your track detection algorithm into
    sigma = 0.3125 / 100 / 3.46
    lat_err = car.get_lateral_error() + np.random.normal(0.0, sigma)

    line0_err = self.get_line_camera_error(car.get_line_camera_image(0))
    line1_err = self.get_line_camera_error(car.get_line_camera_image(1))

    # lat_err = line0_err
    # lat_err = line0_err / 100
    # Proportional gain in steering control (degrees) / lateral error (meters)
    #TODO change kp constant
    kp = 350
    kd = 2500
    if self.last_err is None:
        steer_angle = -kp * lat_err
        self.last_err = lat_err/20
    else:
        self.last_err = (lat_err - self.last_err) / 20
        steer_angle = -kp * lat_err + kd * self.last_err
    # steer_angle = kp * lat_err
    steer_angle = car.set_steering(steer_angle)

    # Constant speed for now. You can tune this and/or implement advanced
    # controllers.
    # TODO: SET SPEED HERE
    car.set_speed(1)

    # Print out debugging info
    pos = car.get_position()
    vel = car.get_velocity()
    vel = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
    print('t=%6.3f (x=%5.2f, y=%5.2f, sp=%5.2f): lat_err=%5.2f, line0_err=%3f, steer_angle=%3.1f'
          % (time, pos[0], pos[1], vel,
             lat_err, (line0_err or 0), steer_angle))
    if csvfile is not None:
      csvfile.writerow({'t': time, 'x': pos[0], 'y': pos[1], 'speed': vel,
                        'lat_err': lat_err,
                        'line0_err': (line0_err or ""),
                        'line1_err': (line1_err or ""),
                        'steer_angle': steer_angle
                        })

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='ee192 Python V-REP Car controller.')
  parser.add_argument('--iterations', metavar='i', type=int, default=500,
                      help='number of control iterations to run')
  parser.add_argument('--synchronous', metavar='p', type=bool, default=False,
                      help="""enable synchronous mode, forcing the simulator to
                      operate in lockstep with the simulator - potentially
                      increases accuracy / repeatability at the cost of
                      performance""")
  parser.add_argument('--restart', metavar='r', type=bool, default=False,
                      help="""whether to restart the simulation if a simulation
                      is currently running""")
  parser.add_argument('--csvfile', metavar='c', default=None,
                      help='csv filename to log to')
  parser.add_argument('--csvfile_overwrite', metavar='c', type=bool, default=False,
                      help='overwrite the specified csvfile without warning')
  args = parser.parse_args()

  # Check that we won't overwrite an existing csvfile before mucking with the
  # simulator.
  if (args.csvfile is not None and os.path.exists(args.csvfile)
      and not args.csvfile_overwrite):
    print("csvfile '%s' already exists: aborting." % args.csvfile)
    sys.exit()

  # Terminate any existing sessions, just in case.
  vrep.simxFinish(-1)

  # Stop the existing simulation if requested. Not using a separate
  # VRepInterface (and hence VREP API client id) seems to cause crashes.
  if args.restart:
    with VRepInterface.open() as vr:
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)

  # Open a V-REP API connection and get the car.
  with VRepInterface.open() as vr:
    if args.synchronous:
      vr.simxSynchronous(1)

    vr.simxStartSimulation(vrep.simx_opmode_oneshot_wait)

    car = Car(vr)
    assignment = SimulationAssignment(vr, car)
    assignment.setup_car(vr, car)

    csvfile = None
    if args.csvfile:
      # Dirty hack to get this (potentially) working in Python 2 and 3
      if sys.version_info.major < 3:
        csvfile = open(args.csvfile, 'wb')
      else:
        csvfile = open(args.csvfile, 'w', newline='')

      fieldnames = ['t', 'x', 'y', 'speed', 'lat_err', 'line0_err', 'line1_err',
                    'steer_angle']
      fielddict = {}
      for fieldname in fieldnames:
        fielddict[fieldname] = fieldname
      csvfile = csv.DictWriter(csvfile, fieldnames=fieldnames)
      csvfile.writerow(fielddict)

    try:
      for i in range(0, args.iterations):
        assignment.control_loop(vr, car, csvfile)

        # Advance to the next frame
        if args.synchronous:
          vr.simxSynchronousTrigger()

      print("Finished %i control iterations: pausing simulation."
            % args.iterations)
      vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
    except KeyboardInterrupt:
      # Allow a keyboard interrupt to break out of the loop while still shutting
      # down gracefully.
      print("KeyboardInterrupt: pausing simulation.")
      vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
