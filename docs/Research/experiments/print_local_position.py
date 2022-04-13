#!/bin/python3
import itertools
import time

from pymavlink import mavutil


def main():
    print("Connecting...")
    drone = mavutil.mavlink_connection("udp:localhost:14580")
    drone.wait_heartbeat()
    print("Connected!")
    for i in itertools.count(1):
        input("Press enter for new line")
        print(drone.recv_match(condition="False"))
        update_time = time.time()
        try:
            print(f"[{i}] - local position: {drone.messages['LOCAL_POSITION_NED']}")
        except KeyError:
            print(f"[{i}] - No local position.")
        armed = drone.motors_armed()
        while not armed:
            print(f"[{time.time() - update_time:.5f}] - armed: {armed}")
            print(drone.recv_match(condition="False"))
            armed = drone.motors_armed()

if __name__ == "__main__":
    main()

