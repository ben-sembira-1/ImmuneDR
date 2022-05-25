from datetime import timedelta, datetime
import logging
from typing import Union, Optional, Set

from async_state_machine import StateMachine
from async_state_machine.state_machine import StateName


class StateMachineError(AssertionError):
    pass


def run_until(
    sm: StateMachine,
    target: Union[
        StateName,
        Set[StateName],
    ],
    error_states: Optional[Set[StateName]] = None,
    timeout: Optional[timedelta] = None,
) -> None:
    """
    Runs the state machine until it reaches a target state or an error state.
    Raises `StateMachineError` if an error state is reached.
    """
    if isinstance(target, set):
        targets = target
    else:
        targets = {target}
    error_states = error_states or set()

    sm.on_state_change(lambda s: logging.info(f"Reached state: {str(s.name)}"))
    start_time = datetime.now()
    while sm.current_state.name not in targets | error_states:
        if timeout is not None and (datetime.now() - start_time > timeout):
            raise TimeoutError(
                f"State machine timed out while waiting for state(s) {target}"
            )
        sm.tick()

    if sm.current_state.name in error_states:
        raise StateMachineError(f"Reached error state {sm.current_state.name}")
