#! /usr/bin/env python3

from dataclasses import dataclass

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard

fake_run: bool = False


@dataclass
class CombatRule:
    health_limit: int = 100
    bullet_limit: int = 20


@dataclass
class CruiseMethod:
    method_name: str = ""
    waypoints: list[float, float] = []


def setup_cruise_methods():
    pass


Blackboard.set("health", 0)
Blackboard.set("bullet", 0)
Blackboard.set("cruise_method", "occupation")


def update_blackboard(combat_status: str):
    pass


class MoveTo(Behaviour):
    def __init__(self) -> None:
        pass

    def update(self) -> None:
        pass


def entrypoint() -> None:
    while True:
        continue


if __name__ == "__main__":
    print("Hello World!!")
