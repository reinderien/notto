#!/usr/bin/python3 -OO
import sys
from heapq import heappop, heappush
from io import StringIO
from itertools import chain, islice
from pathlib import Path
from sys import stdin, stdout
from typing import Iterator, Optional, TextIO

import tkinter as tk

from main import (
    EDGE, SPEED,
    OptimisedWaypoint, Waypoint, time_to
)


class Solver:
    HEAD = OptimisedWaypoint.with_cost(Waypoint(x=0, y=0))
    TAIL = Waypoint(x=EDGE, y=EDGE)

    def __init__(self) -> None:
        self.total_penalty = 0
        self.opt_heap = [self.HEAD]
        self.cost_acceptable = float('inf')
        self.cost_best: Optional[float] = None
        self.cost_min_best: float = self.HEAD.cost_min
        self.cost: Optional[float] = None
        self.to_exceed: Optional[float] = None
        self.to_prune: Optional[set[Waypoint]] = None
        self.waypoints: list[Waypoint] = [self.HEAD.waypoint]
        self.visited: Optional[Waypoint] = None
        self.skip_from: Optional[OptimisedWaypoint] = None
        self.new_opt: Optional[OptimisedWaypoint] = None
        self.best_opt = self.HEAD
        self.final_time: Optional[float] = None

    def prune(self) -> Iterator['Solver']:
        self.to_prune = {
            ow.waypoint
            for ow in self.opt_heap
            if ow.cost_min > self.to_exceed
        }

        while self.opt_heap and self.opt_heap[0].cost_min > self.to_exceed:
            yield self
            popped = heappop(self.opt_heap)
            self.to_prune.remove(popped.waypoint)
        
        self.to_prune = None

    def get_best_cost(self) -> Iterator['Solver']:
        self.cost_best = float('inf')
        for self.skip_from in self.opt_heap:
            self.cost = self.skip_from.cost_to(self.visited)
            self.cost_best = min(self.cost_best, self.cost)
            yield self
        self.skip_from = None
        self.cost = None

    def feed(self, visited: Waypoint) -> Iterator['Solver']:
        self.visited = visited
        self.waypoints.append(visited)
        yield self

        self.total_penalty += visited.penalty

        yield from self.get_best_cost()

        self.new_opt = OptimisedWaypoint.with_cost(waypoint=visited, cost_best=self.cost_best)
        assert self.new_opt.is_sane
        yield self

        if self.cost_acceptable >= self.new_opt.cost_min:
            if self.cost_min_best >= self.new_opt.cost_min:
                self.best_opt = self.new_opt
                self.cost_min_best = self.new_opt.cost_min
                self.cost_acceptable = self.new_opt.cost_max
                self.to_exceed = self.cost_acceptable
                yield from self.prune()

            heappush(self.opt_heap, self.new_opt)

        self.new_opt = None

    def finish(self) -> Iterator['Solver']:
        self.visited = self.TAIL
        self.waypoints.append(self.TAIL)
        yield from self.get_best_cost()
        self.final_time = self.cost_best + self.total_penalty

    def iterate_solve(self, in_: TextIO, n: int) -> Iterator['Solver']:
        yield self

        for line in islice(in_, n):
            visited = Waypoint.from_line(line)
            assert visited.is_sane
            yield from self.feed(visited)

        yield from self.finish()


class AnimateContext:
    SCALE = 8

    def __init__(self, in_: TextIO, n: int) -> None:
        self.solver = Solver()
        self.steps: Iterator[Solver] = chain(self.solver.iterate_solve(in_, n), self.solver.finish())

        w = self.SCALE*(EDGE + 1)
        self.tk = tk.Tk()
        self.tk.geometry(f'{w}x{w}+50+50')
        self.canvas = tk.Canvas(
            self.tk, borderwidth=0, background='#082030',
        )
        self.canvas.pack(expand=True, fill='both')
        self.waypoint_oval_ids: dict[Waypoint, int] = {}

        self.reachable_ids: list[int] = []
        self.comparison_ray_id: Optional[int] = None
        self.path_size = 0

        self.draw()

    def draw(self) -> None:
        step: Optional[Solver] = next(self.steps, None)
        if step is None:
            return

        self.create_full_path(step)
        self.create_waypoints(step)
        self.colour_waypoints(step)
        self.draw_reachable(step)
        self.draw_comparison(step)
        self.tk.after(500, self.draw)

    def create_full_path(self, step: Solver) -> None:
        missing = step.waypoints[self.path_size:]
        for source, dest in zip(missing, missing[1:]):
            self.canvas.create_line(
                self.SCALE*source.x, self.SCALE*source.y,
                self.SCALE*dest.x,   self.SCALE*dest.y,
                width=1, fill='#203440',  # dark blue
            )
        self.path_size = len(step.waypoints)-1

    def create_waypoints(self, step: Solver) -> None:
        for waypoint in (
            step.HEAD.waypoint,
            step.visited,
        ):
            if waypoint is not None and waypoint not in self.waypoint_oval_ids:
                penalty = waypoint.penalty
                if penalty < 1:
                    penalty = 100  # Endpoint

                cx = waypoint.x*self.SCALE
                cy = waypoint.y*self.SCALE
                radius = penalty*self.SCALE/20
                oval_id = self.canvas.create_oval(
                    cx-radius, cy-radius,
                    cx+radius, cy+radius,
                    width=0,
                )
                self.waypoint_oval_ids[waypoint] = oval_id

    def colour_waypoints(self, step: Solver) -> None:
        for waypoint, oval_id in tuple(self.waypoint_oval_ids.items()):
            if waypoint in (step.to_prune or ()):
                fill = '#952d1a'  # dark red
            elif step.skip_from and waypoint == step.skip_from.waypoint:
                fill = '#93cadb'  # light blue
            elif step.best_opt and waypoint == step.best_opt.waypoint:
                fill = '#8916a6'  # purple
            elif step.new_opt and waypoint == step.new_opt.waypoint:
                fill = '#dfc400'  # gold
            elif waypoint == step.visited:
                fill = '#6fdc83'  # light green
            elif waypoint in {
                ow.waypoint for ow in step.opt_heap
            }:
                fill = '#577d94'  # grey blue
            else:
                outline = '#203440'  # dark blue
                self.canvas.itemconfig(oval_id, outline=outline, fill='', width=2)
                continue

            self.canvas.itemconfig(oval_id, fill=fill)

    def draw_reachable(self, step: Solver) -> None:
        while self.reachable_ids:
            self.canvas.delete(self.reachable_ids.pop())

        source = step.best_opt
        if step.to_prune:
            target = step.opt_heap[0]
        else:
            target = step.new_opt

        if not (source and target):
            return

        # cost = distance/speed + best_cost - penalty + delay
        # (acceptable_cost - invariant)*speed = reachable distance
        reachable_dist = (step.cost_acceptable - target.cost_invariant) * SPEED
        radius = max(5., reachable_dist*self.SCALE)

        if step.to_prune or step.cost_acceptable < target.cost_min:
            outline = '#d33a23'  # red
        else:
            outline = '#76d323'  # green

        candidates = (
            (time_to(target.waypoint.x-x, target.waypoint.y-y) * SPEED, x, y)
            for x, y in (
                (0, 0),
                (0, EDGE),
                (EDGE, 0),
                (EDGE, EDGE),
            )
        )
        min_dist, x, y = min(candidates)

        cx = x*self.SCALE
        cy = y*self.SCALE
        self.reachable_ids.append(self.canvas.create_oval(
            cx-radius, cy-radius,
            cx+radius, cy+radius,
            width=2, outline=outline,
        ))

    def draw_comparison(self, step: Solver) -> None:
        if self.comparison_ray_id:
            self.canvas.delete(self.comparison_ray_id)

        if step.to_prune:
            source = step.best_opt.waypoint
            target = step.opt_heap[0].waypoint
            outline = '#d33a23'  # red
        elif step.skip_from:
            source = step.visited
            target = step.skip_from.waypoint
            outline = '#93cadb'  # light blue
        elif step.new_opt:
            source = step.best_opt.waypoint
            target = step.new_opt.waypoint
            outline = '#dfc400'  # gold
        else:
            return

        self.comparison_ray_id = self.canvas.create_line(
            source.x*self.SCALE, source.y*self.SCALE,
            target.x*self.SCALE, target.y*self.SCALE,
            width=2, fill=outline,
        )

    def run(self) -> None:
        self.tk.mainloop()

    def finish(self) -> float:
        return self.solver.final_time


def process_stream(in_: TextIO, out: TextIO) -> None:
    while True:
        n = int(next(in_))
        if n == 0:
            break

        animate = AnimateContext(in_, n)
        animate.run()
        time = animate.finish()
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
