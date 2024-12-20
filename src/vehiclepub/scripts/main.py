#!/usr/bin/env python

import glob
import os
import sys
import time
import math
import argparse
import logging
from numpy import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

# ROS imports
import rospy
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header
from vehiclepub.msg import VehicleInfo, VehicleInfoArray # Assuming custom_msgs/VehicleInfo is defined

def publish_all_vehicles_info(vehicles, pub, collision_detected):
    vehicle_info_array = VehicleInfoArray()
    
    for i, vehicle in enumerate(vehicles):
        vehicle_info = VehicleInfo()
        
        # Populate the VehicleInfo message
        vehicle_info.header.stamp = rospy.Time.now()
        
        transform = vehicle.get_transform()
        vehicle_info.pose.position.x = transform.location.x
        vehicle_info.pose.position.y = transform.location.y
        vehicle_info.pose.position.z = transform.location.z

        yaw_in_radians = math.radians(transform.rotation.yaw)
        vehicle_info.pose.orientation.x = transform.rotation.roll
        vehicle_info.pose.orientation.y = transform.rotation.pitch
        vehicle_info.pose.orientation.z = yaw_in_radians

        # Get the vehicle's bounding box to determine size
        bounding_box = vehicle.bounding_box
        vehicle_info.size.x = bounding_box.extent.x * 2  # Convert half-extent to full size
        vehicle_info.size.y = bounding_box.extent.y * 2
        vehicle_info.size.z = bounding_box.extent.z * 2
        
        # Check for collision
        if collision_detected[i]:
            print("Collision detected for vehicle at position: x={}, y={}, z={}".format(
                transform.location.x, transform.location.y, transform.location.z))
            vehicle_info.collision_detected = True
        
        vehicle_info_array.vehicles.append(vehicle_info)
    
    pub.publish(vehicle_info_array)

def setup_collision_sensor(vehicle, collision_detected, index):
    # Add collision sensor to the vehicle
    blueprint = vehicle.get_world().get_blueprint_library().find('sensor.other.collision')
    collision_sensor = vehicle.get_world().spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)

    # Define the callback to update collision detection status
    def callback(event):
        collision_detected[index] = True

    collision_sensor.listen(callback)
    return collision_sensor

def main(argv):
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=10, type=int, help='number of vehicles (default: 10)')
    argparser.add_argument('-w', '--number-of-walkers', metavar='W', default=50, type=int, help='number of walkers (default: 50)')
    argparser.add_argument('--safe', action='store_true', help='avoid spawning vehicles prone to accidents')
    argparser.add_argument('--filterv', metavar='PATTERN', default='vehicle.*', help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument('--filterw', metavar='PATTERN', default='walker.pedestrian.*', help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument('--tm-port', metavar='P', default=8000, type=int, help='port to communicate with TM (default: 8000)')
    argparser.add_argument('--sync', action='store_true', help='Synchronous mode execution')
    argparser.add_argument('--hybrid', action='store_true', help='Enable hybrid mode')
    argparser.add_argument('-s', '--seed', metavar='S', type=int, help='Random device seed')
    argparser.add_argument('--car-lights-on', action='store_true', default=False, help='Enable car lights')
    argparser.add_argument('--scene', default='long', help='scene <long/compare>')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    # Initialize ROS node
    rospy.init_node('carla_vehicle_publisher', anonymous=True)
    vehicle_info_pub = rospy.Publisher('/static_obstacle/vehicle_info', VehicleInfoArray, queue_size=10)

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter('vehicle.nissan.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = []

        if args.scene == 'long':
            spawn_points.append(carla.Transform(carla.Location(x=123.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=103.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))

            spawn_points.append(carla.Transform(carla.Location(x=193.9, y=230.74, z=0.4), carla.Rotation(yaw=-90)))
            # spawn_points.append(carla.Transform(carla.Location(x=192.9, y=190.74, z=0.4), carla.Rotation(yaw=240)))
            spawn_points.append(carla.Transform(carla.Location(x=190.5, y=190.74, z=0.4), carla.Rotation(yaw=240)))
            spawn_points.append(carla.Transform(carla.Location(x=189.6, y=210.74, z=0.4), carla.Rotation(yaw=90)))

            # spawn_points.append(carla.Transform(carla.Location(x=189.3, y=111.5, z=0.4), carla.Rotation(yaw=230)))
            # spawn_points.append(carla.Transform(carla.Location(x=189.1, y=111.7, z=0.4), carla.Rotation(yaw=230))) #100% 100%
            spawn_points.append(carla.Transform(carla.Location(x=189.2, y=111.6, z=0.4), carla.Rotation(yaw=230))) #

            spawn_points.append(carla.Transform(carla.Location(x=123.4, y=105.0, z=0.4), carla.Rotation(yaw=180)))
            spawn_points.append(carla.Transform(carla.Location(x=103.4, y=105.0, z=0.4), carla.Rotation(yaw=180)))
            spawn_points.append(carla.Transform(carla.Location(x=83.4, y=105.0, z=0.4), carla.Rotation(yaw=180)))
        elif args.scene == 'compare':
            spawn_points.append(carla.Transform(carla.Location(x=72.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))
        elif args.scene == 'success1':
            spawn_points.append(carla.Transform(carla.Location(x=93.32, y=305.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=108.32, y=303.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=123.32, y=305.74, z=0.4), carla.Rotation(yaw=0)))
        elif args.scene == 'success2':
            spawn_points.append(carla.Transform(carla.Location(x=88.32, y=305.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=108.32, y=303.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=128.32, y=305.74, z=0.4), carla.Rotation(yaw=0)))
        elif args.scene == 'success3':
            spawn_points.append(carla.Transform(carla.Location(x=93.32, y=305.99, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=108.32, y=303.49, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=123.32, y=305.99, z=0.4), carla.Rotation(yaw=0)))

        number_of_spawn_points = len(spawn_points)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        batch = []
        initial_positions = []
        collision_detected = [False] * len(spawn_points)
        collision_sensors = []

        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = blueprints[0]
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            light_state = vls.NONE
            if args.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            batch.append(SpawnActor(blueprint, transform)
                .then(SetVehicleLightState(FutureActor, light_state)))

            initial_positions.append((transform.location.x, transform.location.y, transform.location.z))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % (len(vehicles_list)))

        traffic_manager.global_percentage_speed_difference(30.0)

        for i, vehicle_id in enumerate(vehicles_list):
            vehicle = world.get_actor(vehicle_id)
            sensor = setup_collision_sensor(vehicle, collision_detected, i)
            collision_sensors.append(sensor)

        while not rospy.is_shutdown():
            if args.sync and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
                
            publish_all_vehicles_info([world.get_actor(x) for x in vehicles_list], vehicle_info_pub, collision_detected)

            # Print collision information
            for i, collided in enumerate(collision_detected):
                if collided:
                    logging.info("Vehicle {} has collided.".format(i))


    finally:
        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        for sensor in collision_sensors:
            sensor.destroy()

        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
