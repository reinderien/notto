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
    penalty: Real

    best_time: float = 0
    accrued_penalty: float = 0

    @classmethod
    def from_line(cls, line: str) -> 'Waypoint':
        return cls(*(int(t) for t in line.split()))

    def time_to(self, other: 'Waypoint') -> float:
        distance = sqrt((other.x-self.x)**2 + (other.y-self.y)**2)
        return distance / SPEED

    def __repr__(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3} ap={self.accrued_penalty:3} bt={self.best_time:6.1f}'


def possible_times(waypoints: Sequence[Waypoint], i_visited: int, visited: Waypoint) -> Iterator[float]:
    for skipto in waypoints[i_visited+1:]:
        travel_time = visited.time_to(skipto)
        penalty = DELAY + skipto.accrued_penalty
        cost = travel_time + skipto.best_time + penalty
        yield cost
        skipto.accrued_penalty += visited.penalty


def solve(interior_waypoints: Iterable[Waypoint]) -> float:
    """
    The brute-force approach is O(2^n) in time. This does a faster O(n^2)-time search
    that is O(n) in memory. More optimisation is possible.
    """

    waypoints: tuple[Waypoint, ...] = (
        Waypoint(x=0, y=0, penalty=0),
        *interior_waypoints,
        Waypoint(x=100, y=100, penalty=float('inf')),
    )

    for i_visited in range(len(waypoints)-2, -1, -1):
        visited: Waypoint = waypoints[i_visited]
        visited.best_time = min(possible_times(waypoints, i_visited, visited))
        pprint(waypoints)

    return waypoints[0].best_time


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
