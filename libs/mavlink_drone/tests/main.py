from libs.mavlink_drone.src.drone_controller import Drone


def continue_with_enter(msg: str) -> None:
    input(f"{msg} (press enter to continue)")


def main():
    continue_with_enter("Connect")
    drone = Drone("udpin:localhost:14560", source_system=0)
    continue_with_enter("Land")
    drone.mode = "LAND"
    continue_with_enter("Change mode to guided")
    drone.mode = "GUIDED"
    continue_with_enter("Arm and takeoff")
    drone.rc(throttle=-1)
    drone.arm()
    drone.takeoff(5)
    drone.rc()
    while remote_control_commands := input("rc (p r y t): "):
        drone.rc(*[float(command) for command in remote_control_commands.split()])
    continue_with_enter("Set EKF failsafe to change mode to ALT_HOLD")
    while ekf_action := input("ekf action: "):
        drone.fs_ekf_action = int(ekf_action)
    continue_with_enter("Land")
    drone.mode = "LAND"


if __name__ == '__main__':
    main()
