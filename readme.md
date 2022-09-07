Algorithm
=========

Exponential
-----------

Given that every waypoint requires the choice of skip-or-not, a brute-force implementation costs O(2^n) in time.

Quadratic
---------

The first complexity reduction to O(n^2) comes by:

- traversing, in the outer loop, "visited" waypoints in reverse order
- for each visited waypoint, in an inner loop, traversing all "skip-to" waypoints after the visited waypoint in forward
  order
- for every skip-to waypoint, adding the visited-waypoint penalty to its "accrued penalty" sum
- interpreting this choice as skipping every waypoint from the visited waypoint up to the skip-to waypoint, and then 
  accepting the stored best path time from that skip-to waypoint through to the end
- calculating the cost of this choice as

    (travel distance from visited to skip_to)/speed + fixed_delay (10s) + (best_time of skip_to waypoint) + accrued_penalty

- storing the minimum cost of the inner loop to the best cost attribute of the visited waypoint
- iterating the outer loop all the way to the beginning, and returning the best cost of the first waypoint.

Linear
------

The next complexity reduction to O(n) requires cost analysis. The outer loop cannot be reduced but the inner loop can. 
Separate from the waypoint structure an optimised-waypoint structure that stores the best cost and the accrued penalty.
In its cost expression, identify which components vary based on the location of the source waypoint and which components
are invariant:

    travel_time + fixed_delay + best_time + accrued_penalty
    variant       invariant    invariant    invariant

The only term that varies is the distance, and this distance has bounds:

    minimum distance = 1 (due to waypoint uniqueness specification)
    maximum distance = 100*sqrt(2) ~ 141.4
    minimum time = 0.5
    maximum time ~ 70.7

Due to those bounds, and due to the invariant costs being overwhelmingly greater for non-trivial input, the inner-loop
sequence of optimised waypoints can be sorted by invariant cost and pruned. In the outer loop, sort the sequence of
optimised waypoints in increasing order by the sum of invariant costs of each waypoint; the fixed delay can be omitted.
In C++, rather than sorting, use a self-sorted container, `multimap` with the key being the invariant cost. In Python
the best we can do is logarithmic `bisect.insort()` into a list.

Then consider the maximum cost delta between the start and end of this sequence where, beyond this delta, it will be 
impossible to see a cost smaller than at the start. This worst-case calculation is done by assuming that the distance 
from "visited" to the first "skip-to" is maximal, and the distance from "visited" to the last "skip-to" before pruning
is minimal; effectively:

    index        time    invariant   cost
    -----        ----    ---------   ----
    first        70.7    a         70.7+a
    ...
    prune-here    0.5    b          0.5+b

    70.7 - 0.5 + a < b

This pruning step is highly effective, and even for large input produces an optimised waypoint sequence that never
exceeds 20 elements. `multimap` is typically implemented as a balanced binary tree and has O(log(m)) insertion and
lookup, `m` as the length of the optimised waypoint sequence. Since the map size remains small, the inner loop becomes
O(1) amortised over `n`.

Make the pruning even more aggressive: rather than a fixed maximum distance, use the maximum distance from the first 
waypoint based on the distance to the farthest spatial bound (0 or 100) over both dimensions.

Negative penalty accrual
------------------------

Up to now, the inner loop that is aggregated by a `min()` required adding a penalty to every skip-to waypoint in the
optimised waypoint sequence. This expense can be reduced from O(m) to O(1), avoiding `m` mutations (indeed, avoiding
mutation of any kind on the optimised waypoint structures): do not accrue positive penalties in the inner loop. Instead,
assign the current visited waypoint's penalty as a negative cost component rather than a positive cost component. All
cost ordering - variant and invariant - remains in increasing order; all relative cost distances remain the same as they
were; but costs tend to become negative. At the end of the outer loop, compensate by adding the total penalty of all
waypoints to the cost of the first waypoint.

Norm simplification
-------------------

There are two potential simplifications to the Frobenius norm calculation, neither actually helping performance.

The first is to use the built-in `std::hypot`. This ends up being far slower because of the care it takes in mitigating
edge conditions that are irrelevant for this application.

The second is, during the inner min() aggregation loop, solve the inequality comparing the two costs for the norm's
radicand. This allows sqrt() to only be needed once on every in-loop decrease in the score rather than every single
iteration. Unfortunately, any time saved in avoiding root calls is negated by the cost of the surrounding machinery.

Space
-----

Loading all waypoints is O(n) in space. Whereas this could be reduced to O(1) by parsing the file in reverse, this
complexity is not needed due to the upper bound of the problem. Since all waypoints must be spatially unique and since
coordinates can only exist in the closed interval [1, 99], the maximum number of waypoints is

    (99 - 1 + 1)² = 9801

or a file size of about 85 kB. All waypoints can be trivially held in memory even on embedded systems.

Parsing
-------

`callgrind` profiling reveals a surprising bottleneck, the `istream`-based parse of the input file. The naïve approach
of `in >> x >> y >> penalty` is so slow that, for 1,000,000 waypoints, it accounts for some 70% of program execution
time. The alternative approach written in `Waypoint::read` gets a line and manually applies `stoi()` to the triple,
bringing the parse step down to about 45% of execution time. Though the actual problem scale does not call for this,
further improvements could use manually manipulated buffers and/or producer/consumer parallelism.

Performance
===========

On an 11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz, compiling the C++ implementation with

    g++ -Ofast -s -march=native --std=c++20 -Wall -Wextra -pedantic

and benchmarking three cases, all times in approximate milliseconds:

    Dataset             C++20      CPython 3.10.5
    -------             -----      --------------
    all testcases           4                  29
    9801 waypoints          7                  80
    1,000,000 waypoints   140                5733
