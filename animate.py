#!/usr/bin/env python3
import sys
from bisect import bisect
from io import StringIO
from itertools import chain
from math import sqrt
from pathlib import Path
from sys import stdin, stdout
from typing import Iterator, Optional, NamedTuple, Sequence, TextIO, Iterable

import tkinter as tk


NDEBUG = True

DELAY = 10        # seconds
SPEED = 2         # metres per second
EDGE = 100        # metres

# These are theoretical bounds; we get narrower than this
DISTANCE_MIN = 1
DISTANCE_MAX = sqrt(2) * 100
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


class WaypointReader:
    def __init__(self, in_: TextIO) -> None:
        self.lines = iter(reversed(in_.readlines()))
        self.case_count = 0
        self.next_waypoint: Optional[Waypoint] = None
        assert not self.advance_state()

    @property
    def stream_end(self) -> bool:
        return self.lines is None

    def advance_state(self) -> bool:
        line = next(self.lines, None)
        if line is None:
            self.lines = None
            return False

        fields = [int(f) for f in line.split()]

        if len(fields) == 1:
            count, = fields
            if count == self.case_count:
                self.case_count = 0
                return False
            raise ValueError()

        self.next_waypoint = Waypoint(*fields)
        self.case_count += 1
        return True

    def get_next(self) -> Waypoint:
        return self.next_waypoint


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


class Solver:
    HEAD = Waypoint(x=0, y=0)
    TAIL = OptimisedWaypoint.with_cost(Waypoint(x=100, y=100))

    def __init__(self) -> None:
        self.opt_waypoints = [self.TAIL]
        self.acceptable_cost = float('inf')
        self.total_penalty = 0
        self.waypoints: list[Waypoint] = [self.TAIL.waypoint]
        self.best_cost: Optional[float] = None
        self.cost: Optional[float] = None
        self.prune_from: Optional[int] = None
        self.visited: Optional[Waypoint] = None
        self.skipto: Optional[OptimisedWaypoint] = None
        self.new_opt: Optional[OptimisedWaypoint] = None
        self.final_time: Optional[float] = None

    def prune(self) -> Iterator['Solver']:
        # opt_waypoints must be in increasing order of invariant cost
        self.prune_from = bisect(a=self.opt_waypoints, x=self.acceptable_cost, key=OptimisedWaypoint.sort_key)
        if not NDEBUG:
            assert self.prune_from > 0
        while len(self.opt_waypoints) > self.prune_from:
            yield self
            self.opt_waypoints.pop()
        self.prune_from = None

    def get_best_cost(self) -> Iterator['Solver']:
        self.best_cost = float('inf')
        for self.skipto in self.opt_waypoints:
            if not NDEBUG:
                assert self.skipto.is_sane
            self.cost = self.skipto.cost_from(self.visited)
            self.best_cost = min(self.best_cost, self.cost)
            yield self
        self.skipto = None
        self.cost = None

    @property
    def is_sane(self) -> bool:
        return all(ow.is_sane for ow in self.opt_waypoints)

    def feed(self, visited: Waypoint) -> Iterator['Solver']:
        self.visited = visited
        self.waypoints.append(visited)
        yield self

        if not NDEBUG:
            assert visited.is_sane
            assert self.is_sane

        self.total_penalty += visited.penalty

        yield from self.get_best_cost()

        self.new_opt = OptimisedWaypoint.with_cost(waypoint=visited, best_cost=self.best_cost)
        if not NDEBUG:
            assert self.new_opt.is_sane
        yield self

        if self.new_opt.cost_min <= self.acceptable_cost and self.new_opt.emplace(self.opt_waypoints):
            self.acceptable_cost = self.new_opt.cost_max
            yield from self.prune()

        self.new_opt = None

    def finish(self) -> Iterator['Solver']:
        self.visited = self.HEAD
        self.waypoints.append(self.HEAD)
        yield from self.get_best_cost()
        self.final_time = self.best_cost + self.total_penalty

    def iterate_solve(self, reader: WaypointReader) -> Iterator['Solver']:
        yield self

        while reader.advance_state():
            yield from self.feed(reader.get_next())

        if not reader.stream_end:
            yield from self.finish()


class AnimateContext:
    SCALE = 8

    def __init__(self, reader: WaypointReader) -> None:
        self.solver = Solver()
        self.steps: Iterator[Solver] = chain(self.solver.iterate_solve(reader), self.solver.finish())

        w = self.SCALE*EDGE
        self.tk = tk.Tk()
        self.tk.geometry(f'{w}x{w}+50+50')
        self.canvas = tk.Canvas(
            self.tk, borderwidth=0, background='#082030',
        )
        self.canvas.pack(expand=True, fill='both')
        self.waypoint_oval_ids: dict[Waypoint, int] = {}

        self.draw()

    def draw(self) -> None:
        step: Optional[Solver] = next(self.steps, None)
        if step is None:
            return

        # First, draw all points if they're not already in the dict
        for waypoint in (
            step.visited,
            step.TAIL.waypoint,
        ):
            if waypoint is not None and waypoint not in self.waypoint_oval_ids:
                penalty = waypoint.penalty
                if penalty < 1:
                    penalty = 100  # Endpoint

                cx = (waypoint.x + 0.5)*self.SCALE
                cy = (waypoint.y + 0.5)*self.SCALE
                radius = penalty*self.SCALE/20
                oval_id = self.canvas.create_oval(
                    cx-radius, cy-radius,
                    cx+radius, cy+radius,
                    width=0, fill='#203440')
                self.waypoint_oval_ids[waypoint] = oval_id

        for waypoint, oval_id in tuple(self.waypoint_oval_ids.items()):
            if step.prune_from is not None and waypoint in {
                ow.waypoint for ow in step.opt_waypoints[step.prune_from:]
            }:
                fill = '#952d1a'  # dark red
            elif step.new_opt and waypoint == step.new_opt.waypoint:
                fill = '#dfc400'  # gold
            elif step.skipto and waypoint == step.skipto.waypoint:
                fill = '#93cadb'  # light blue
            elif waypoint == step.visited:
                fill = '#6fdc83'  # light green
            elif waypoint in {
                ow.waypoint for ow in step.opt_waypoints
            }:
                fill = '#203440'  # dark blue
            else:
                self.canvas.delete(oval_id)
                del self.waypoint_oval_ids[waypoint]
                continue

            self.canvas.itemconfig(oval_id, fill=fill)

        self.tk.after(2000, self.draw)

    def run(self) -> None:
        self.tk.mainloop()

    def finish(self) -> float:
         return self.solver.final_time


def process_stream(in_: TextIO, out: TextIO) -> None:
    reader = WaypointReader(in_)

    times = []

    while not reader.stream_end:
        animate = AnimateContext(reader)
        animate.run()
        times.append(animate.finish())

    for time in times[::-1]:
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
