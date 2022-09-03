#!/usr/bin/env python3
from dataclasses import dataclass
from io import StringIO
from math import sqrt
from numbers import Real
from pprint import pprint
from sys import stdin, stdout
from typing import TextIO, Iterable, Iterator, Sequence

SPEED = 2
DELAY = 10


@dataclass
class Waypoint:
    x: Real
    y: Real
    penalty: Real = 0

    @classmethod
    def from_line(cls, line: str) -> 'Waypoint':
        return cls(*(int(t) for t in line.split()))

    def time_to(self, other: 'Waypoint') -> float:
        distance = sqrt((other.x-self.x)**2 + (other.y-self.y)**2)
        return distance / SPEED

    def __repr__(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3}'


@dataclass
class OptimisedWaypoint:
    waypoint: Waypoint
    accrued_penalty: float = 0
    best_time: float = 0

    def __repr__(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return f'{self.waypoint} ap={self.accrued_penalty:3} bt={self.best_time:6.1f}'

    def cost_for(self, visited: Waypoint) -> float:
        travel_time = visited.time_to(self.waypoint)
        penalty = DELAY + self.accrued_penalty
        cost = travel_time + self.best_time + penalty
        return cost


def possible_times(opt_waypoints: Sequence[OptimisedWaypoint], visited: Waypoint) -> Iterator[float]:
    for skipto in opt_waypoints:
        yield skipto.cost_for(visited)
        skipto.accrued_penalty += visited.penalty


def solve(interior_waypoints: Iterable[Waypoint]) -> float:
    """
    The brute-force approach is O(2^n) in time. This does a faster O(n^2)-time search
    that is O(n) in memory. More optimisation is possible.
    """

    waypoints: tuple[Waypoint, ...] = (
        Waypoint(x=0, y=0),
        *interior_waypoints,
        Waypoint(x=100, y=100),
    )

    opt_waypoints: list[OptimisedWaypoint] = [
        OptimisedWaypoint(waypoint=waypoints[-1]),
    ]

    for i_visited in range(len(waypoints)-2, -1, -1):
        visited: Waypoint = waypoints[i_visited]

        best_time = min(possible_times(opt_waypoints, visited))
        opt_waypoints.insert(0, OptimisedWaypoint(
            waypoint=visited, best_time=best_time,
        ))
        pprint(opt_waypoints)

    return opt_waypoints[0].best_time


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(in_.readline())
        if n < 1:
            break

        waypoints = (Waypoint.from_line(in_.readline()) for _ in range(n))
        time = solve(waypoints)
        out.write(f'{time:.3f}\n')


def test() -> None:
    for case in ('small', 'medium', 'large'):
        with open(f'sample_input_{case}.txt') as in_, \
             StringIO() as out:
            process_stream(in_, out)
            out.seek(0)
            print(out.getvalue())

            with open(f'sample_output_{case}.txt') as out_exp:
                for line in out_exp:
                    actual = next(out)
                    assert line == actual


def main() -> None:
    process_stream(stdin, stdout)


if __name__ == '__main__':
    main()
