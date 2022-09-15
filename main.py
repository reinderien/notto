#!/usr/bin/python3 -OO
"""
This is an implementation meant to mirror that in main.cpp fairly closely.
It's much slower, but can sometimes be easier to debug during algorithmic development.
asserts get disabled in the .pyc when it's compiled with -OO, similar to NDEBUG in C++.
"""

import sys
from heapq import heappop, heappush
from io import StringIO
from itertools import islice
from math import sqrt
from pathlib import Path
from sys import stdin, stdout
from typing import Iterator, NamedTuple, Sequence, TextIO


DELAY = 10        # seconds
SPEED = 2         # metres per second
EDGE = 100        # metres

DISTANCE_MIN = 0
DISTANCE_MAX = EDGE * sqrt(2)
TIME_MIN = DISTANCE_MIN / SPEED
TIME_MAX = DISTANCE_MAX / SPEED


def time_to(dx: int, dy: int) -> float:
    assert -EDGE <= dx <= EDGE
    assert -EDGE <= dy <= EDGE

    time = sqrt(dx**2 + dy**2) / SPEED
    assert TIME_MIN <= time <= TIME_MAX

    return time


def coord_min(x: int) -> int:
    return min(EDGE - x, x)


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
        return time_to(other.x - self.x, other.y - self.y)

    @property
    def time_min(self) -> float:
        return time_to(coord_min(self.x), coord_min(self.y))

    @property
    def time_max(self) -> float:
        return time_to(coord_max(self.x), coord_max(self.y))

    def __str__(self) -> str:
        return f'({self.x},{self.y}) penalty={self.penalty}'

    @property
    def is_sane(self) -> bool:
        return (0 <= self.x <= EDGE and
                0 <= self.y <= EDGE)


class OptimisedWaypoint(NamedTuple):
    waypoint: Waypoint
    cost_invariant: float
    cost_min: float

    @classmethod
    def with_cost(cls, waypoint: Waypoint, cost_best: float = 0) -> 'OptimisedWaypoint':
        cost_invariant = cost_best - waypoint.penalty + DELAY
        return cls(
            waypoint=waypoint, cost_invariant=cost_invariant,
            cost_min=waypoint.time_min + cost_invariant,
        )

    def cost_to(self, visited: Waypoint) -> float:
        time = visited.time_to(self.waypoint)
        return time + self.cost_invariant

    @property
    def cost_max(self) -> float:
        return self.waypoint.time_max + self.cost_invariant

    def __str__(self) -> str:
        return (
            f'{self.waypoint}'
            f' cost_inv={self.cost_invariant:.3f}'
            f' cost_min={self.cost_min:.3f}'
        )

    @property
    def is_sane(self) -> bool:
        return self.waypoint.is_sane

    def __lt__(self, other: 'OptimisedWaypoint') -> bool:
        # heappush and heappop have no key argument, so we need __lt__.
        # heapq creates min-heaps, but we need a max-heap so we invert this predicate.
        return self.cost_min >= other.cost_min


def prune(opt_heap: list[OptimisedWaypoint], to_exceed: float) -> None:
    while opt_heap and opt_heap[0].cost_min > to_exceed:
        heappop(opt_heap)


def possible_costs(visited: Waypoint, opt_heap: Sequence[OptimisedWaypoint]) -> Iterator[float]:
    for skip_from in opt_heap:
        yield skip_from.cost_to(visited)


def solve(in_: TextIO, n: int) -> float:
    total_penalty = 0
    head = OptimisedWaypoint.with_cost(Waypoint(x=0, y=0))
    opt_heap = [head]
    cost_acceptable = float('inf')
    cost_min_best = head.cost_min

    for line in islice(in_, n):
        visited = Waypoint.from_line(line)
        assert visited.is_sane
        total_penalty += visited.penalty

        cost_best = min(possible_costs(visited, opt_heap))
        new_opt = OptimisedWaypoint.with_cost(waypoint=visited, cost_best=cost_best)
        assert new_opt.is_sane

        if cost_acceptable >= new_opt.cost_min:
            if cost_min_best >= new_opt.cost_min:
                cost_min_best = new_opt.cost_min
                cost_acceptable = new_opt.cost_max
                prune(opt_heap, cost_acceptable)

            heappush(opt_heap, new_opt)

    tail = Waypoint(x=EDGE, y=EDGE)
    cost_best = min(possible_costs(tail, opt_heap))
    return cost_best + total_penalty


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(next(in_))
        if n == 0:
            break

        time = solve(in_, n)
        out.write(f'{time:.3f}\n')


def test() -> None:
    parent = Path('samples')
    for case in ('small', 'medium', 'large'):
        with (parent / f'sample_input_{case}.txt').open() as in_, StringIO() as out:
            process_stream(in_, out)
            out.seek(0)

            with (parent / f'sample_output_{case}.txt').open() as out_exp:
                for line in out_exp:
                    actual = next(out)
                    print(f'{line.rstrip()} == {actual.rstrip()}')
                    assert line == actual


def main() -> None:
    if '-t' in sys.argv:
        test()
    else:
        process_stream(stdin, stdout)


if __name__ == '__main__':
    main()
