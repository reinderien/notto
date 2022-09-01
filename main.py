#!/usr/bin/env python3

from io import StringIO
from math import sqrt
from numbers import Real
from sys import stdin, stdout
from typing import TextIO, NamedTuple, Iterable, Iterator

SPEED = 2
DELAY = 10


class Waypoint(NamedTuple):
    x: Real
    y: Real
    penalty: Real

    @classmethod
    def from_line(cls, line: str) -> 'Waypoint':
        return cls(*(int(t) for t in line.split()))

    def time_to(self, other: 'Waypoint') -> float:
        distance = sqrt((other.x-self.x)**2 + (other.y-self.y)**2)
        return distance / SPEED


FIRST = Waypoint(x=0, y=0, penalty=float('inf'))
LAST = Waypoint(x=100, y=100, penalty=float('inf'))


def solve(interior_waypoints: Iterable[Waypoint]) -> float:
    """
    The brute-force approach is O(2^n) in time. This does a faster O(n^2)-time search
    that is O(n) in memory. More optimisation is possible.
    """

    waypoints: tuple[Waypoint, ...] = (FIRST, *interior_waypoints, LAST)
    best_times = [0.] * len(waypoints)

    for i_visited in range(len(waypoints)-2, -1, -1):

        def possible_times() -> Iterator[float]:
            penalties = DELAY
            for i_skipto in range(i_visited + 1, len(waypoints)):
                travel_time = waypoints[i_visited].time_to(waypoints[i_skipto])
                yield travel_time + best_times[i_skipto] + penalties
                penalties += waypoints[i_skipto].penalty

        best_times[i_visited] = min(possible_times())

    return best_times[0]


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
