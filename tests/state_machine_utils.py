from typing import Union, Optional, Set

from async_state_machine import StateMachine
from async_state_machine.state_machine import StateName


class StateMachineError(Exception):
    pass


def run_until(
    sm: StateMachine,
    target: Union[
        StateName,
        Set[StateName],
    ],
    error_states: Optional[Set[StateName]] = None,
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

    while sm.current_state.name not in targets | error_states:
        sm.tick()

    if sm.current_state.name in error_states:
        raise StateMachineError(f"Reached error state {sm.current_state.name}")
