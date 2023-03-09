"""
@ Jiaxin Yang 20230308
This node is Carla Client
function:
it will spawn ego vehicle: vehicle.lincoln.mkz_2017
and spawn npc vehicle on adjacent lane
if cet ego vehicle's location directly
transform = carla.Transform(carla.Location(x=10, y=20), carla.Rotation(yaw=30))
vehicle.set_transform(transform)
message:
it publish sensor data to perception module
it act as a service server to supply waypoint list to MPC controller
it act as a service client to get vehicle state from Speedgoat node
"""
# python library
import random
import pygame
import numpy as np
import time
import csv
# carla library
import carla

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.load_world('Bend01')

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)

# Set a seed so behaviour can be repeated if necessary
traffic_manager.set_random_device_seed(0)
random.seed(0)

# spawn ego vehicle at left lane
blueprint_library = world.get_blueprint_library()
ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2017')
ego_vehicle_bp.set_attribute('color', '0, 255, 0')
transform = carla.Transform(carla.Location(x=454.495026, y=318.445648, z=4.000000), carla.Rotation(yaw=188))
# transform = random.choice(world.get_map().get_spawn_points()) # for random spawn_point
# print(transform.location)
ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
time.sleep(1.5) # you should set a delay for ego vehicle to spawn to fix the spectator

# set up a spectator at driver's view
spectator = world.get_spectator()
transform = ego_vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-0.22,y=0.3,z=1.31),
                                        transform.rotation)) 

# spawn npc vehicles at left lane
npc_spawn_point = carla.Transform(ego_vehicle.get_transform().location + carla.Location(x=-8,y=-3.35,z=4), ego_vehicle.get_transform().rotation)
# npc_spawn_point = random.choice(world.get_map().get_spawn_points()) 
npc_vehicle_bp = blueprint_library.filter('model3')
npc_vehicle = world.spawn_actor(npc_vehicle_bp[0], npc_spawn_point)

# Get a starting waypoint near the npc_vehicle
map = world.get_map()
# topology = map.get_topology()  # test the road info
# print(topology)
start_waypoint = map.get_waypoint(npc_vehicle.get_location(), project_to_road=True, \
                                  lane_type=(carla.LaneType.Driving))
# Set the distance between vehicles
distance = 10
# Set the number of vehicles to spawn
num_vehicles = 20
# Create a list of spawn points
spawn_points = []
# Loop through the number of vehicles
for i in range(num_vehicles):
    # Get the next waypoint at the given distance
    next_waypoint = start_waypoint.next(distance)[0]
    # Get a spawn point from the waypoint transform
    spawn_point = carla.Transform(next_waypoint.transform.location + carla.Location(z=4), next_waypoint.transform.rotation)
    # Append the spawn point to the list
    spawn_points.append(spawn_point)
    # Update the start waypoint to the next waypoint
    start_waypoint = next_waypoint

# Spawn vehicles at the spawn points
blueprint_library = world.get_blueprint_library()
# vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
vehicle_bp = blueprint_library.filter('model3')[0]
vehicles = [npc_vehicle]
for spawn_point in spawn_points:
    # print(spawn_point)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)  # try_spawn_actor() return None if spawn cause collision
    vehicles.append(vehicle)

# Set target speed for npv vehicles (in m/s)
# traffic_manager = client.get_trafficmanager(8000)
target_speed = 60 / 3.6
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    traffic_manager.vehicle_percentage_speed_difference(vehicle, target_speed - vehicle.get_velocity().length())
    traffic_manager.auto_lane_change(vehicle, False)


# while True:
#     ego_vehicle.set_autopilot(True)
#     print("control npc by tracffic_manager")

# 1. set up a camera to follow ego vehicle
# 2. define a callback function for camera.listen(...) to 
# 3. render the pixel data of one camera to the PyGame interface. 
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0, 255, (height, width, 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image)

def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:, :, :3]
    img = img[:, :, ::-1] # reverse the third dimension for openCV
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

# Control Object to manage vehicle controls
class ControlObject(object):
    def __init__(self, veh):
        # Conrol parameters to store the control state
        self._vehicle = veh
        self._throttle = False
        self._brake = False
        self._steer = None
        self._steer_cache = 0
        # A carla.VehicleControl object is needed to alter the vehicle's control state
        self._control = carla.VehicleControl()
    
    # Check for key press events in the PyGame window and define the control state
    def parse_control(self, event):
        if event.type == pygame.KEYDOWN: # is there any key to be pressed?
            if event.key == pygame.K_RETURN:  # is the key pressed the Enter?
                self._vehicle.set_autopilot(False)
            if event.key == pygame.K_UP:
                self._throttle = True
            if event.key == pygame.K_DOWN:
                self._brake = True
            if event.key == pygame.K_RIGHT:
                self._steer = 1
            if event.key == pygame.K_LEFT:
                self._steer = -1
        if event.type == pygame.KEYUP:   # is the key released?
            if event.key == pygame.K_UP:
                self._throttle = False
            if event.key == pygame.K_DOWN:
                self._brake = False
                self._control.reverse = False
            if event.key == pygame.K_RIGHT:
                self._steer = None
            if event.key == pygame.K_LEFT:
                self._steer = None
    
    # Process the current control state, change the control parameter
    # if the key remains pressed
    def process_control(self):
        if self._throttle:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0
        
        if self._brake:
            # If the down arrow is held down when the car is stationary, switch to reverse
            if self._vehicle.get_velocity().length() < 0.01 and not self._control.reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            else:
                self._control.throttle = 0.0
                self._control.brake = min(self._control.brake + 0.3, 1)
        else:
            self._control.brake = 0.0


# Initialise the camera floating behind the vehicle
# camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))  
camera_init_trans = carla.Transform(carla.Location(x=-0.22,y=0.3,z=1.31), ego_vehicle.get_transform().rotation)    
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
try:
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to = ego_vehicle)
    print(f"Camera spawned successfully with id {camera.id}")
except RuntimeError as e:
    print(f"Camera spawn failed: {e}")

# Start camera with PyGame callback
camera.listen(lambda image: pygame_callback(image, renderObject))

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for rendering and vehicle control
renderObject = RenderObject(image_w, image_h)
controlObject = ControlObject(ego_vehicle)

# Initialise the PyGame interface. This will call up a new window for PyGame.
# Initialise the display
pygame.init()
gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
# Draw black to the display
gameDisplay.fill((0,0,0))
gameDisplay.blit(renderObject.surface, (0,0))  # copy surface to (0,0) position
pygame.display.flip() # update the display



# Game loop
crashed = False

while not crashed:
    # Advance the simulation time
    world.tick()
    # Update the display
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
    # Process the current control state
    controlObject.process_control()
    # Collect key press events
    for event in pygame.event.get():
        # If the window is closed, break the while loop
        if event.type == pygame.QUIT:
            crashed = True

        # Parse effect of key press event on control state
        controlObject.parse_control(event)
        if event.type == pygame.KEYUP:
            # TAB key switches vehicle
            if event.key == pygame.K_TAB:
                ego_vehicle.set_autopilot(True)
                # ego_vehicle = random.choice(vehicles)         
                # Ensure vehicle is still alive (might have been destroyed)
                if ego_vehicle.is_alive:
                    # Stop and remove the camera
                    camera.stop()
                    camera.destroy()

                    # Spawn new camera and attach to new vehicle
                    controlObject = ControlObject(ego_vehicle)
                    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
                    camera.listen(lambda image: pygame_callback(image, renderObject))

                    # Update PyGame window
                    gameDisplay.fill((0,0,0))               
                    gameDisplay.blit(renderObject.surface, (0,0))
                    pygame.display.flip()

# Stop camera and quit PyGame after exiting game loop
camera.stop()
pygame.quit()


    






