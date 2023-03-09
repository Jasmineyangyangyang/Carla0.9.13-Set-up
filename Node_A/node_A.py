"""
@ Jiaxin Yang 20230308
This node is Carla Client
function:
it will spawn ego vehicle: vehicle.lincoln.mkz_2017
and spawn npc vehicle on adjacent lane

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
# settings = world.get_settings()
# settings.synchronous_mode = True # Enables synchronous mode
# settings.fixed_delta_seconds = 0.05
# world.apply_settings(settings)

# # Set up the TM in synchronous mode
# traffic_manager = client.get_trafficmanager()
# traffic_manager.set_synchronous_mode(True)

# # Set a seed so behaviour can be repeated if necessary
# traffic_manager.set_random_device_seed(0)
# random.seed(0)

# spawn ego vehicle at left lane
blueprint_library = world.get_blueprint_library()
ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2017')
ego_vehicle_bp.set_attribute('color', '255, 0, 0')
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
vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
vehicle_bp = blueprint_library.filter('model3')[0]
vehicles = [npc_vehicle]
for spawn_point in spawn_points:
    # print(spawn_point)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    vehicles.append(vehicle)

# Set target speed for npv vehicles (in m/s)
tm = client.get_trafficmanager(8000)
target_speed = 60 / 3.6
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    tm.vehicle_percentage_speed_difference(vehicle, target_speed - vehicle.get_velocity().length())
    tm.auto_lane_change(vehicle, False)

# time.sleep(180)
# while True:
#     print("control npc by tracffic_manager")







