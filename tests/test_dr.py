import logging
from datetime import timedelta

import pymavlink.mavutil

from drones.testing import SimulationDroneClient, TurnOffGps
from tests.dr_state_machine import DRStateNames, get_dr_state_machine
from tests.state_machine_utils import run_until


def test_drone_dr(
    mavlink_connection: pymavlink.mavutil.mavfile,
    flying_sim_drone: SimulationDroneClient,
) -> None:
    dr_state_machine = get_dr_state_machine(flying_sim_drone)
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
