import logging
from datetime import timedelta

import pymavlink.mavutil
from drones.geo import GeoLocation
from drones.latest_messages import LatestMessagesCache
from pymavlink.dialects.v20.ardupilotmega import MAVLink_sim_state_message

from drones.testing import SimulationDroneClient, TurnOffGps
from tests.dr_state_machine import DRStateNames, get_dr_state_machine
from tests.state_machine_utils import run_until

upper_left = GeoLocation(latitude=-35.36151054, longitude=149.16379178)
home = GeoLocation(latitude=-35.36326086, longitude=149.16523056)


def test_drone_dr(
    mavlink_connection: pymavlink.mavutil.mavfile,
    flying_sim_drone: SimulationDroneClient,
) -> None:
    latest_messages = LatestMessagesCache(flying_sim_drone)

    dr_target = upper_left

    dr_state_machine = get_dr_state_machine(flying_sim_drone, dr_target=dr_target)
    # Disabling gps should cause the EKF to fail, eventually triggering DR
    logging.info("Disabling gps")
    TurnOffGps()(mavlink_connection)
    logging.info("Waiting to lose GPS lock")
    run_until(
        dr_state_machine,
        target=DRStateNames.GUIDED_NO_GPS,
        error_states={DRStateNames.ERROR},
        timeout=timedelta(seconds=5),
    )
    logging.info("Started DR")
    for state in [
        DRStateNames.CLIMBING,
        DRStateNames.TURNING,
        DRStateNames.INBOUND,
        DRStateNames.DESCENDING,
        DRStateNames.LANDED,
    ]:
        logging.info(f"Waiting for state {state}")
        run_until(
            dr_state_machine,
            target=state,
            error_states={DRStateNames.ERROR, DRStateNames.IN_THE_AIR},
            timeout=timedelta(seconds=15),
        )
        logging.info(f"Reached state {state}")

    end_location_in_sim = GeoLocation.from_message(
        latest_messages.get_latest_message(MAVLink_sim_state_message)
    )
    assert (
        end_location_in_sim.distance_to(upper_left) < 100
    ), f"Finished at {end_location_in_sim}, which is {end_location_in_sim.distance_to(dr_target)} [m] from destination"
