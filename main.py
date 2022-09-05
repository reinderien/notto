#!/usr/bin/env python3
import sys
from io import StringIO
from math import sqrt
from pathlib import Path
from sys import stdin, stdout
from typing import Iterable, Iterator, NamedTuple, Sequence, TextIO

DELAY = 10
SPEED = 2
MIN_DISTANCE = 1
MAX_DISTANCE = sqrt(2) * 100
MIN_TIME = MIN_DISTANCE / SPEED
MAX_TIME = MAX_DISTANCE / SPEED


class Waypoint(NamedTuple):
    x: int
    y: int
    penalty: int = 0

    @classmethod
    def from_line(cls, line: str) -> 'Waypoint':
        return cls(*(int(t) for t in line.split()))

    def time_to(self, other: 'Waypoint') -> float:
        distance = sqrt((other.x-self.x)**2 + (other.y-self.y)**2)
        return distance / SPEED

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3}'


class OptimisedWaypoint(NamedTuple):
    waypoint: Waypoint
    accrued_penalty: int = 0
    best_cost: float = 0

    def __str__(self) -> str:
        return f'{self.waypoint} ap={self.accrued_penalty:3} bt={self.best_cost:6.1f}'

    def cost_for(self, visited: Waypoint) -> float:
        travel_time = visited.time_to(self.waypoint)
        return travel_time + self.best_cost - self.accrued_penalty + DELAY

    def sort_key(self) -> float:
        return self.invariant_cost

    @property
    def invariant_cost(self) -> float:
        return self.best_cost - self.accrued_penalty


def prune(opt_waypoints: list[OptimisedWaypoint]) -> None:
    # opt_waypoints must be in increasing order of invariant cost
    to_exceed = opt_waypoints[0].invariant_cost + MAX_TIME - MIN_TIME

    while opt_waypoints[-1].invariant_cost > to_exceed:
        opt_waypoints.pop()


def possible_costs(opt_waypoints: Sequence[OptimisedWaypoint], visited: Waypoint) -> Iterator[float]:
    for skipto in opt_waypoints:
        yield skipto.cost_for(visited)


def solve(interior_waypoints: Iterable[Waypoint]) -> float:
    waypoints = (
        Waypoint(x=0, y=0),
        *interior_waypoints,
        Waypoint(x=100, y=100),
    )

    opt_waypoints = [OptimisedWaypoint(waypoint=waypoints[-1])]
    total_penalty = 0

    for visited in waypoints[-2::-1]:
        total_penalty += visited.penalty
        best_cost = min(possible_costs(opt_waypoints, visited))
        opt_waypoints.append(OptimisedWaypoint(waypoint=visited, best_cost=best_cost, accrued_penalty=visited.penalty))
        opt_waypoints.sort(key=OptimisedWaypoint.sort_key)
        prune(opt_waypoints)

    return best_cost + total_penalty


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(in_.readline())
        if n < 1:
            break

        waypoints = (Waypoint.from_line(in_.readline()) for _ in range(n))
        cost = solve(waypoints)
        out.write(f'{cost:.3f}\n')


def test() -> None:
    parent = Path('samples')
    for case in ('small', 'medium', 'large'):
        with (parent / f'sample_input_{case}.txt').open() as in_, StringIO() as out:
            process_stream(in_, out)
            out.seek(0)
            print(out.getvalue())

            with (parent / f'sample_output_{case}.txt').open() as out_exp:
                for line in out_exp:
                    actual = next(out)
                    assert line == actual


def main() -> None:
    if '-t' in sys.argv:
        test()
    else:
        process_stream(stdin, stdout)


if __name__ == '__main__':
    main()
