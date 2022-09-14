Algorithm
=========

Exponential
-----------

Given that every waypoint requires the choice of skip-or-not, a brute-force implementation costs O(2^n) in time.

Quadratic
---------

The first complexity reduction to O(n²) comes by:

- traversing, in the outer loop, "visited" waypoints
- for each visited waypoint, in an inner loop, traversing all "skip-from" waypoints to the visited waypoint
- for every skip-from waypoint, adding the visited-waypoint penalty to its "accrued penalty" sum
- interpreting this choice as skipping every waypoint from the skip-from waypoint up to the visited waypoint, and then 
  accepting the stored best path time from that skip-from waypoint through to the beginning
- calculating the cost of this choice as

    (travel distance from skip_from to visited)/speed + fixed_delay (10s) + (best_time of skip_from waypoint) + accrued_penalty

- storing the minimum cost of the inner loop to the best cost attribute of the visited waypoint
- iterating the outer loop all the way to the end, and returning the best cost of the final waypoint.

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

We can do better than those theoretical bounds, because individual waypoints can have distance minima and maxima based
on their position:

    minimum distance = sqrt(min(x, 100-x)² + min(y, 100-y)²)
    maximum distance = sqrt(max(x, 100-x)² + max(y, 100-y)²)

Due to those bounds, and due to the invariant costs being overwhelmingly greater for non-trivial input, the inner-loop
sequence of optimised waypoints can be sorted by invariant cost and pruned. In the outer loop, sort the sequence of
optimised waypoints in increasing order by the sum of invariant costs of each waypoint. In C++, rather than sorting, use
a self-sorted container, `multimap` with the key being the invariant cost. In Python the best we can do is logarithmic 
`bisect.insort()` into a list.

Then consider the maximum cost delta between the start and end of this sequence where, beyond this delta, it will be 
impossible to see a cost smaller than at the start. This worst-case calculation is done by assuming that the distance 
from "visited" to the first "skip-from" is maximal, and the distance from "visited" to the last "skip-from" before 
pruning is minimal; effectively:

    index       invariant   cost
    -----       ---------   ----
    first           inv_a   max_time_a + inv_a
    ...
    prune_here      inv_b   min_time_b + inv_b

    max_time_a + inv_a < min_time_b + inv_b

This pruning step is highly effective, and even for large input produces an optimised waypoint sequence that never
exceeds 20 elements. `multimap` is typically implemented as a balanced binary tree and has O(log(m)) insertion and
lookup, `m` as the length of the optimised waypoint sequence. Since the map size remains small, the inner loop becomes
O(1) amortised over `n`.

Negative penalty accrual
------------------------

Up to now, the inner loop that is aggregated by a `min()` required adding a penalty to every skip-from waypoint in the
optimised waypoint sequence. This expense can be reduced from O(m) to O(1), avoiding `m` mutations (indeed, avoiding
mutation of any kind on the optimised waypoint structures): do not accrue positive penalties in the inner loop. Instead,
assign the current visited waypoint's penalty as a negative cost component rather than a positive cost component. All
cost ordering - variant and invariant - remains in increasing order; all relative cost distances remain the same as they
were; but costs tend to become negative. At the end of the outer loop, compensate by adding the total penalty of all
waypoints to the cost of the final waypoint.

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

Since all waypoints must be spatially unique and since coordinates can only exist in the open interval (0, 100), the
maximum number of waypoints should be

    (100 - 1)² = 9801

or a file size of about 85 kB. All waypoints can be trivially held in memory even on embedded systems. (Despite the
specification promising waypoint uniqueness, the sample data violate this constraint.)

This algorithm need not hold all waypoints, since it can process the input with O(1) space. However, it is simpler to
load all content into memory and then parse and process.

Parsing
-------

`callgrind` profiling reveals a surprising bottleneck: if `istream`-based operations are used to parse the file, that is
the dominant cost. The naïve approach of `in >> x >> y >> penalty` is so slow that, for 1,000,000 waypoints, it accounts
for some 76% of program execution time. The alternative approach written in `WaypointReader` is to read the entire file
into memory and then do a low-level parse. This brings the parse step down to about 23% of execution time.

Performance
===========

On an 11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz, compiling the C++ implementation with

    g++ -Ofast -s -march=native --std=c++20 -Wall -Wextra -pedantic

and benchmarking three cases, all times in approximate milliseconds:

    Dataset             C++20      CPython 3.10.5
    -------             -----      --------------
    all testcases           3                  30
    9801 waypoints          5                  79
    1,000,000 waypoints    75                5626
