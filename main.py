#!/usr/bin/env python3
import sys
from bisect import insort, bisect
from io import StringIO
from math import sqrt
from pathlib import Path
from sys import stdin, stdout
from typing import Iterator, NamedTuple, Sequence, TextIO

DELAY = 10
SPEED = 2
DISTANCE_MIN = 1
TIME_MIN = DISTANCE_MIN / SPEED
# DISTANCE_MAX = sqrt(2) * 100     # we can get narrower than this
# TIME_MAX = DISTANCE_MAX / SPEED


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

    @property
    def time_max(self) -> float:
        distance_max = sqrt(
            max(self.x, 100 - self.x) ** 2 +
            max(self.y, 100 - self.y) ** 2
        )
        return distance_max / SPEED

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3}'


class OptimisedWaypoint(NamedTuple):
    waypoint: Waypoint
    penalty: int = 0
    best_cost: float = 0

    def __str__(self) -> str:
        return f'{self.waypoint} ap={self.penalty:3} bt={self.best_cost:6.1f}'

    def cost_from(self, visited: Waypoint) -> float:
        travel_time = visited.time_to(self.waypoint)
        return travel_time + self.invariant_cost + DELAY

    def sort_key(self) -> float:
        return self.invariant_cost

    @property
    def invariant_cost(self) -> float:
        return self.best_cost - self.penalty


def prune(opt_waypoints: list[OptimisedWaypoint]) -> None:
    # opt_waypoints must be in increasing order of invariant cost
    to_exceed = opt_waypoints[0].invariant_cost + opt_waypoints[0].waypoint.time_max - TIME_MIN
    prune_from = bisect(opt_waypoints, to_exceed, key=OptimisedWaypoint.sort_key)
    del opt_waypoints[prune_from:]


def possible_costs(opt_waypoints: Sequence[OptimisedWaypoint], visited: Waypoint) -> Iterator[float]:
    for skipto in opt_waypoints:
        yield skipto.cost_from(visited)


def solve(waypoints: Sequence[Waypoint]) -> float:
    opt_waypoints = [OptimisedWaypoint(waypoint=waypoints[-1])]
    total_penalty = 0

    for visited in waypoints[-2::-1]:
        total_penalty += visited.penalty
        best_cost = min(possible_costs(opt_waypoints, visited))
        new_opt = OptimisedWaypoint(waypoint=visited, best_cost=best_cost, penalty=visited.penalty)
        insort(opt_waypoints, new_opt, key=OptimisedWaypoint.sort_key)
        prune(opt_waypoints)

    return best_cost + total_penalty


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(in_.readline())
        if n < 1:
            break
        waypoints = (
            Waypoint(x=0, y=0),
            *(Waypoint.from_line(in_.readline()) for _ in range(n)),
            Waypoint(x=100, y=100),
        )
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
