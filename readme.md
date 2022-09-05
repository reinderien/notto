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
In C++, rather than sorting, use a self-sorted container, `multiset`.

Then consider the maximum cost delta between the start and end of this sequence where, beyond this delta, it will be 
impossible to see a cost smaller than before it. This worst-case calculation is done by assuming that the distance from 
"visited" to the first "skip-to" is maximal, and the distance from "visited" to the last "skip-to" before pruning is 
minimal; effectively:

    index        time    invariant   cost
    -----        ----    ---------   ----
    first        70.7    a         70.7+a
    ...
    prune-here    0.5    b          0.5+b

    70.7 - 0.5 + a < b

This pruning step is highly effective, and even for large input produces an optimised waypoint sequence that never
exceeds 20 elements. Since `multiset` is typically implemented as a balanced binary tree and has O(log(m)) insertion,
`m` as the length of the optimised waypoint sequence, and since the set size remains small, the inner loop becomes O(1)
amortised over `n`.

Negative penalty accrual
------------------------

Up to now, the inner loop that is aggregated by a `min()` required adding a penalty to every skip-to waypoint in the
optimised waypoint sequence. This expense can be reduced from O(m) to O(1) and avoiding `m` mutations (indeed, avoiding
mutation of any kind on the optimised waypoint structures): do not accrue positive penalties in the inner loop. Instead,
assign the current visited waypoint's penalty as a negative cost component rather than a positive cost component. All
cost ordering - variant and invariant - remains in increasing order; all relative cost distances remain the same as they
were; but costs tend to become negative. At the end of the outer loop, compensate by adding the total penalty of all
waypoints to the cost of the first waypoint.

Space
-----

Loading all waypoints is O(n) in space. Whereas this could be reduced to O(1) by parsing the file in reverse, this
complexity is not needed due to the upper bound of the problem. Since all waypoints must be spatially unique and since
coordinates can only exist in the closed interval [1, 99],

    max waypoints = (99 - 1 + 1)² = 9801

All waypoints can be trivially held in memory even on embedded systems.

Performance
===========

On an 11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz, compiling the C++ implementation with

    g++ -O3 -s -march=native -fomit-frame-pointer --std=c++20 -Wall -Wextra -pedantic

processing the maximum well-formed input size (9801 waypoints) takes ~7ms. Processing a large, 250,000-waypoint file
mal-formed due to ignoring the waypoint uniqueness constraint takes ~45 ms.

Running through the entire provided test suite takes ~30 ms with the Python implementation and ~4 ms with the C++
implementation.