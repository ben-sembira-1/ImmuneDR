from libs.immune_dr.src.auto_pilot import AutoPilot
from libs.mavlink_drone.src.drone_controller import Drone


def main():
    autopilot = AutoPilot(Drone("udpin:localhost:14560", source_system=0))
    autopilot.start_mission()


if __name__ == '__main__':
    main()
