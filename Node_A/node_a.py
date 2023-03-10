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
world = client.load_world('Bend02')

# Set up the simulator in synchronous mode
original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05 # 50ms, None means variable time-step
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
ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
time.sleep(1.5) # you should set a delay for ego vehicle to spawn to fix the spectator
ego_vehicle.set_autopilot(True, 8000)

actor_list = [ego_vehicle]
# spawn npc vehicles at left lane, notice that all spawn points can not be calculate by something moves in simulation in synchronous mode
map = world.get_map()
start_waypoint = map.get_waypoint(carla.Location(x=454.495026, y=318.445648, z=4.000000)+ carla.Location(x=-8,y=-3.35,z=0.0),\
                                   project_to_road=True, \
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
vehicles = []
for spawn_point in spawn_points:
    # print(spawn_point)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)  # try_spawn_actor() return None if spawn cause collision
    vehicles.append(vehicle)
    actor_list.append(vehicle)

# Set target speed for npv vehicles (in m/s)
# traffic_manager = client.get_trafficmanager(8000)
target_speed = 60 / 3.6
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    traffic_manager.vehicle_percentage_speed_difference(vehicle, target_speed - vehicle.get_velocity().length())
    traffic_manager.auto_lane_change(vehicle, False)


class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

# Initialise the camera floating behind the vehicle
# camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))  
camera_init_trans = carla.Transform(carla.Location(x=0.10,y=-0.35,z=1.20), carla.Rotation())    
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
try:
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to = ego_vehicle)
    print(f"Camera spawned successfully with id {camera.id}")
except RuntimeError as e:
    print(f"Camera spawn failed: {e}")

sensor_list = [camera]  # for destyoy

# Start camera with PyGame callback
camera.listen(lambda image: pygame_callback(image, renderObject))

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for rendering and vehicle control
renderObject = RenderObject(image_w, image_h)

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

    # set up a spectator at driver's view
    spectator = world.get_spectator()
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                            carla.Rotation(pitch=-90))) 
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

    

# Stop camera and quit PyGame after exiting game loop
camera.stop()
world.apply_settings(original_settings)
print('destroying actors')
client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
for sensor in sensor_list:
    sensor.destroy()
print('done.')

pygame.quit()
