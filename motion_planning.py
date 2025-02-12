import argparse
import time
import msgpack
from enum import Enum, auto
import networkx as nx
import numpy as np
import numpy.linalg as LA

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
# Utility functions from the planning_utils
from planning_utils import a_star_graph, closest_point, prune_path, heuristic, create_graph


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        self.send_waypoints()
        
    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Waypoints", self.waypoints)
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        latitude, longitude = self.get_home_position()

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(longitude, latitude, 0.0)

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        current_global_position = np.array([self._longitude, self._latitude, self._altitude])
        self._north, self._east, self._down = global_to_local(current_global_position, self.global_home)

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Get a graph representation with edges for a particular altitude and safety margin around obstacles
        graph, north_offset, east_offset = create_graph(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # TODO: convert start position to current position rather than map center
        current_position = self.local_position
        start = (-north_offset + int(current_position[0]) , -east_offset + int(current_position[1]))

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_longitude = -122.397745
        goal_latitude =  37.793837

        #goal_longitude = -122.394324
        #goal_latitude =  37.791684
    
        #goal_longitude =  -122.401216
        #goal_latitude =  37.796713

        target_location = global_to_local([goal_longitude, goal_latitude, 0], self.global_home)
        goal = (int(-north_offset + target_location[0]), int(-east_offset + target_location[1]))
        
        # Compute the closest points to the start and goal positions
        closest_start = closest_point(graph, (-north_offset + int(current_position[0]) , -east_offset + int(current_position[1])))
        closest_goal = closest_point(graph, (-north_offset + int(target_location[0]), -east_offset + int(target_location[1])))
        
        print("Local Start and Goal : ", start, goal)

        # Run A* to find a path from start to goal
        path, _ = a_star_graph(graph, closest_start, closest_goal)
        print("Found a path of length : ", len(path))

        # Check for collinearity between points and prune the obtained path
        path = prune_path(path)
        # Convert path to waypoints
        self.waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # TODO: send waypoints to simulator
        print("Sending waypoints to the simulator")
        self.send_waypoints()

    def get_home_position(self):
        with open('colliders.csv') as f:
            latLongLine = f.readlines()[0]
        
        strings = latLongLine.split(', ')
    
        latitude = strings[0].split()[1]
        longitude = strings[1].split()[1]

        return float(latitude), float(longitude)

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
