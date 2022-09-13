#!/usr/bin/env python3
import sys
from bisect import bisect
from io import StringIO
from math import sqrt
from pathlib import Path
from sys import stdin, stdout
from typing import Iterator, NamedTuple, Sequence, TextIO

NDEBUG = True

DELAY = 10        # seconds
SPEED = 2         # metres per second
EDGE = 100        # metres

# These are theoretical bounds; we get narrower than this
DISTANCE_MIN = 1
DISTANCE_MAX = sqrt(2) * EDGE
TIME_MIN = DISTANCE_MIN / SPEED
TIME_MAX = DISTANCE_MAX / SPEED


def time_to(dx: int, dy: int) -> float:
    if not NDEBUG:
        assert -EDGE <= dx <= EDGE
        assert -EDGE <= dy <= EDGE

    return sqrt(dx**2 + dy**2) / SPEED


def coord_min(x: int) -> int:
    return max(1, min(EDGE - x, x))


def coord_max(x: int) -> int:
    return max(EDGE - x, x)


class Waypoint(NamedTuple):
    x: int
    y: int
    penalty: int = 0

    @classmethod
    def from_line(cls, line: str) -> 'Waypoint':
        return cls(*(int(t) for t in line.split()))

    def time_to(self, other: 'Waypoint') -> float:
        return time_to(self.x - other.x, self.y - other.y)

    @property
    def time_min(self) -> float:
        return time_to(coord_min(self.x), coord_min(self.y))

    @property
    def time_max(self) -> float:
        return time_to(coord_max(self.x), coord_max(self.y))

    def __str__(self) -> str:
        return f'({self.x:3},{self.y:3}) p={self.penalty:3}'

    @property
    def is_sane(self) -> bool:
        return (0 <= self.x <= EDGE and
                0 <= self.y <= EDGE)


class OptimisedWaypoint(NamedTuple):
    waypoint: Waypoint
    best_cost: float
    invariant_cost: float
    cost_min: float
    penalty: int

    @classmethod
    def with_cost(cls, waypoint: Waypoint, best_cost: float = 0) -> 'OptimisedWaypoint':
        invariant_cost = best_cost - waypoint.penalty + DELAY
        return cls(
            waypoint=waypoint, best_cost=best_cost, invariant_cost=invariant_cost,
            cost_min=waypoint.time_min + invariant_cost, penalty=waypoint.penalty,
        )

    def cost_from(self, visited: Waypoint) -> float:
        time = visited.time_to(self.waypoint)
        if not NDEBUG:
            assert time <= TIME_MAX
        return time + self.invariant_cost

    @property
    def cost_max(self) -> float:
        return self.waypoint.time_max + self.invariant_cost

    def emplace(self, into: list['OptimisedWaypoint']) -> bool:
        i = bisect(a=into, x=self.cost_min, key=OptimisedWaypoint.sort_key)
        into.insert(i, self)
        return i == 0

    def sort_key(self) -> float:
        return self.invariant_cost

    def __str__(self) -> str:
        return f'{self.waypoint} ap={self.penalty:3} bt={self.best_cost:6.1f}'

    @property
    def is_sane(self) -> bool:
        return self.waypoint.is_sane


def prune(opt_waypoints: list[OptimisedWaypoint], to_exceed: float) -> None:
    # opt_waypoints must be in increasing order of invariant cost
    prune_from = bisect(a=opt_waypoints, x=to_exceed, key=OptimisedWaypoint.sort_key)
    if not NDEBUG:
        assert prune_from > 0
    del opt_waypoints[prune_from:]


def possible_costs(visited: Waypoint, opt_waypoints: Sequence[OptimisedWaypoint]) -> Iterator[float]:
    for skipfrom in opt_waypoints:
        if not NDEBUG:
            assert skipfrom.is_sane
        yield skipfrom.cost_from(visited)


class Solver:
    HEAD = OptimisedWaypoint.with_cost(Waypoint(x=0, y=0))
    TAIL = Waypoint(x=EDGE, y=EDGE)

    def __init__(self) -> None:
        self.opt_waypoints = [self.HEAD]
        self.acceptable_cost = float('inf')
        self.total_penalty = 0

    @property
    def is_sane(self) -> bool:
        return all(ow.is_sane for ow in self.opt_waypoints)

    def feed(self, visited: Waypoint) -> None:
        if not NDEBUG:
            assert visited.is_sane
            assert self.is_sane

        self.total_penalty += visited.penalty

        best_cost = min(possible_costs(visited, self.opt_waypoints))
        new_opt = OptimisedWaypoint.with_cost(waypoint=visited, best_cost=best_cost)
        if not NDEBUG:
            assert new_opt.is_sane

        if new_opt.cost_min <= self.acceptable_cost and new_opt.emplace(self.opt_waypoints):
            self.acceptable_cost = new_opt.cost_max
            prune(self.opt_waypoints, self.acceptable_cost)

    def finish(self) -> float:
        visited = self.TAIL
        best_cost = min(possible_costs(visited, self.opt_waypoints))
        return best_cost + self.total_penalty


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(next(in_))
        if n == 0:
            break

        solver = Solver()
        if not NDEBUG:
            assert solver.is_sane

        for _ in range(n):
            solver.feed(Waypoint.from_line(next(in_)))
            if not NDEBUG:
                assert solver.is_sane

        time = solver.finish()
        out.write(f'{time:.3f}\n')


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
