#!/usr/bin/env python3
from dataclasses import dataclass
from io import StringIO
from math import sqrt
from numbers import Real
from sys import stdin, stdout
from typing import Iterable, Iterator, Sequence, TextIO

DELAY = 10
SPEED = 2
MIN_DISTANCE = 1
MAX_DISTANCE = sqrt(2) * 100
MIN_TIME = MIN_DISTANCE / SPEED
MAX_TIME = MAX_DISTANCE / SPEED


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

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3}'


@dataclass
class OptimisedWaypoint:
    waypoint: Waypoint
    accrued_penalty: int = 0
    best_cost: float = 0

    def __str__(self) -> str:
        return f'{self.waypoint} ap={self.accrued_penalty:3} bt={self.best_cost:6.1f}'

    def cost_for(self, visited: Waypoint) -> float:
        travel_time = visited.time_to(self.waypoint)
        return travel_time + self.best_cost + self.accrued_penalty + DELAY

    def sort_key(self) -> int:
        return self.accrued_penalty

    @property
    def invariant_cost(self) -> float:
        return self.accrued_penalty + self.best_cost


def prune(opt_waypoints: list[OptimisedWaypoint]) -> None:
    """
    Since these waypoints are sorted in increasing order of accrued penalty, and the cost formula is
    travel_time + skippedto.best_cost + 10 + accrued_penalty
                  ^---- invariant from here onward
    only the travel time can vary the cost over iterations of the outer loop.
    We need to prune the collection so that if
    - the distance to the first element is maximal, and
    - the distance to the last element is minimal,
    the last element's cost will still not exceed that of the first element.

    index   distance    time   invariant cost
    first   141.42      70.71  a         70.71+a
    ...
    last    1           0.5    b         0.5+b

    70.71 - 0.5 + a < b
    """
    to_exceed = opt_waypoints[0].invariant_cost + MAX_TIME - MIN_TIME

    while opt_waypoints[-1].invariant_cost > to_exceed:
        opt_waypoints.pop()


def possible_costs(opt_waypoints: Sequence[OptimisedWaypoint], visited: Waypoint) -> Iterator[float]:
    for skipto in opt_waypoints:
        yield skipto.cost_for(visited)
        skipto.accrued_penalty += visited.penalty


def solve(interior_waypoints: Iterable[Waypoint]) -> float:
    """
    opt_waypoints, after pruning, stays small (< 20). The min() and sort() over it and the pruning operation are O(1)
    amortised in time and space. The outer loop is then O(n) in time. Since we store all waypoint coordinates after
    parse, that's O(n) in space but could be made O(1) if we were to parse the file in reverse order.
    """

    waypoints = (
        Waypoint(x=0, y=0),
        *interior_waypoints,
        Waypoint(x=100, y=100),
    )

    opt_waypoints = [OptimisedWaypoint(waypoint=waypoints[-1])]

    for i_visited in range(len(waypoints)-2, -1, -1):
        visited: Waypoint = waypoints[i_visited]
        best_cost = min(possible_costs(opt_waypoints, visited))
        opt_waypoints.append(OptimisedWaypoint(waypoint=visited, best_cost=best_cost))
        opt_waypoints.sort(key=OptimisedWaypoint.sort_key)
        prune(opt_waypoints)

    return best_cost


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(in_.readline())
        if n < 1:
            break

        waypoints = (Waypoint.from_line(in_.readline()) for _ in range(n))
        cost = solve(waypoints)
        out.write(f'{cost:.3f}\n')


def test() -> None:
    for case in ('small', 'medium', 'large'):
        with open(f'sample_input_{case}.txt') as in_, StringIO() as out:
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
