#include <algorithm>
#include <cassert>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ranges>
#include <vector>


using namespace std::string_view_literals;


namespace {
    constexpr int
        delay = 10,  // seconds
        speed = 2,   // metres per second
        edge = 100;  // metres

    /*
    These are theoretical bounds; we get narrower than this during the pruning step.
    We cannot use dist_min = 1, due to waypoints such as (4, 2) in
    sample_input_large.txt that violate the uniqueness constraint
    */
    constexpr double
        dist_min = 0,
        dist_max = edge*std::numbers::sqrt2,
        time_min = dist_min / speed,
        time_max = dist_max / speed;

    constexpr std::errc success = std::errc();


    constexpr double time_to(int dx, int dy) {
        assert(-edge <= dx); assert(dx <= edge);
        assert(-edge <= dy); assert(dy <= edge);

        // std::hypot(dx, dy) makes better use of the library but is much slower
        double time = std::sqrt(dx*dx + dy*dy) / speed;
        assert(!std::isnan(time));
        assert(time_min <= time); assert(time <= time_max);

        return time;
    }


    constexpr int coord_min(int x) {
        return std::min(edge-x, x);
    }

    constexpr int coord_max(int x) {
        return std::max(edge-x, x);
    }


    // Direct representation of waypoints parsed from the input
    class Waypoint {
    private:
        int x, y, penalty;

    public:
        constexpr Waypoint(int x, int y, int penalty = 0): x(x), y(y), penalty(penalty) { }

        constexpr double time_to(const Waypoint &other) const {
            return ::time_to(other.x - x, other.y - y);
        }

        constexpr double time_min() const {
            return ::time_to(coord_min(x), coord_min(y));
        }

        constexpr double time_max() const {
            return ::time_to(coord_max(x), coord_max(y));
        }

        void output(std::ostream &out) const {
            out << '(' << x << ',' << y << ") penalty=" << penalty;
        }

        constexpr bool is_sane() const {
            return x >= 0 && x <= edge &&
                   y >= 0 && y <= edge;
        }

        constexpr int get_penalty() const { return penalty; }
    };

    std::ostream &operator<<(std::ostream &out, const Waypoint &w) {
        w.output(out);
        return out;
    }


    // Parser to replace the quite-slow istream << method
    class WaypointReader {
    private:
        static const std::invalid_argument parse_error;

        const std::string body_mem;
        const std::string_view body_view;
        long pos = 0;

        void get_line(const char *&line_start, const char *&line_end) {
            size_t next_pos = body_view.find('\n', pos),
                   substr_len = next_pos - pos;
            std::string_view line = body_view.substr(pos, substr_len);
            line_start = line.data();
            line_end = line_start + substr_len;
            pos = next_pos + 1;
        }

        int get_int(const char *&start, const char *end) {
            int i;
            std::from_chars_result r = std::from_chars(start, end, i);
            if (r.ec != success)
                throw parse_error;
            start = r.ptr + 1;
            return i;
        }

        int get_last_int(const char *start, const char *end) {
            int i;
            std::from_chars_result r = std::from_chars(start, end, i);
            if (r.ec != success || r.ptr != end)
                throw parse_error;
            return i;
        }

    public:
        constexpr WaypointReader(const std::string &body_str):
            body_mem(body_str), body_view(body_mem) {
        }

        static WaypointReader from_stream(std::istream &in) {
            std::ostringstream incopy;
            incopy << in.rdbuf();
            return WaypointReader(incopy.str());
        }

        int get_case_size() {
            const char *start, *end;
            get_line(start, end);
            return get_last_int(start, end);
        }

        Waypoint get_next() {
            const char *start, *end;
            get_line(start, end);
            int x = get_int(start, end),
                y = get_int(start, end),
                penalty = get_last_int(start, end);
            return Waypoint(x, y, penalty);
        }
    };

    const std::invalid_argument WaypointReader::parse_error("Invalid input line");


    // A sidekick to Waypoint that includes optimiser data. Only one "visited" Waypoint is held
    // in memory at a time, but a small handful of OptimisedWaypoints are held in a working heap.
    class OptimisedWaypoint {
    public:
        Waypoint waypoint;
        double cost_invariant,  // Sum of invariant costs incurred by skipping from this waypoint
               cost_min;        // Lowest possible cost incurred by skipping from this waypoint to anywhere

        // cost_best is the cost of the optimal path from the beginning all the way here
        constexpr OptimisedWaypoint(const Waypoint &waypoint, double cost_best = 0):
            waypoint(waypoint), cost_invariant(cost_best - waypoint.get_penalty() + delay),
            cost_min(waypoint.time_min() + cost_invariant) { }

        constexpr OptimisedWaypoint(const OptimisedWaypoint &copy) = default;

        constexpr double cost_to(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            return time + cost_invariant;
        }

        constexpr double cost_max() const {
            return waypoint.time_max() + cost_invariant;
        }

        void output(std::ostream &out) const {
            out << waypoint
                << " cost_inv=" << cost_invariant
                << " cost_min=" << cost_min;
        }

        constexpr bool is_sane() const {
            return waypoint.is_sane();
        }

        constexpr bool operator<(const OptimisedWaypoint &other) const {
            return cost_min < other.cost_min;
        }
    };

    std::ostream &operator<<(std::ostream &out, const OptimisedWaypoint &ow) {
        ow.output(out);
        return out;
    }


    // Erase all heap waypoints whose minimum cost is greater than to_exceed. to_exceed is the maximum cost of the
    // waypoint having the lowest minimum cost of any optimised waypoint in the heap.
    void prune(std::vector<OptimisedWaypoint> &opt_heap, double to_exceed) {
        while (!opt_heap.empty() && opt_heap.front().cost_min > to_exceed) {
            std::pop_heap(opt_heap.begin(), opt_heap.end());
            opt_heap.pop_back();
        }
    }


    constexpr double get_best_cost(
        const Waypoint &visited,
        const std::vector<OptimisedWaypoint> &opt_heap
    ) {
        double cost_best = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skip_from: opt_heap)
            cost_best = std::min(cost_best, skip_from.cost_to(visited));

        assert(cost_best < std::numeric_limits<double>::max());
        return cost_best;
    }


    double solve(WaypointReader &reader, int n) {
        int total_penalty = 0;

        constexpr OptimisedWaypoint head(Waypoint(0, 0));

        // Max-heap of optimised waypoints with the first element
        // guaranteed to have highest minimum possible skip-from cost
        std::vector<OptimisedWaypoint> opt_heap { head };

        // The maximum acceptable cost, set as the maximum possible cost of the lowest-minimum-cost waypoint.
        // Any waypoints costing more than this are discarded.
        double cost_acceptable = std::numeric_limits<double>::max(),
               cost_min_best = head.cost_min;

        for (int i = 0; i < n; ++i) {
            Waypoint visited = reader.get_next();
            assert(visited.is_sane());
            total_penalty += visited.get_penalty();

            double cost_best = get_best_cost(visited, opt_heap);
            OptimisedWaypoint new_opt(visited, cost_best);
            assert(new_opt.is_sane());

            if (cost_acceptable >= new_opt.cost_min) {
                if (cost_min_best >= new_opt.cost_min) {
                    cost_min_best = new_opt.cost_min;
                    cost_acceptable = new_opt.cost_max();

                    // Only prune if the new waypoint has been accepted and has become the lowest-minimum-cost waypoint.
                    // Otherwise, the cost bounds will not have changed.
                    prune(opt_heap, cost_acceptable);
                }

                opt_heap.emplace_back(new_opt);
                std::push_heap(opt_heap.begin(), opt_heap.end());
            }
        }

        constexpr Waypoint tail(edge, edge);
        double cost_best = get_best_cost(tail, opt_heap);

        // Since waypoint costs are calculated with a negative relative penalty,
        // compensate by adding the total penalty to get the true cost
        return cost_best + total_penalty;
    }


    void process_streams(std::istream &in, std::ostream &out) {
        constexpr std::ios::iostate mask = std::ios::failbit | std::ios::badbit;
        in.exceptions(mask);
        out.exceptions(mask);
        out << std::fixed << std::setprecision(3);

        WaypointReader reader = WaypointReader::from_stream(in);

        for (;;) {
            int n = reader.get_case_size();
            if (n == 0) break;

            double time = solve(reader, n);
            out << time << '\n';
        }
    }


    void compare(std::istream &out_exp, std::istream &out_act) {
        for (;;) {
            std::string time_exp, time_act;
            if (!std::getline(out_exp, time_exp)) {
                if (out_exp.eof()) break;
                throw std::ios::failure("getline");
            }
            out_act >> time_act;
            std::cout << time_exp << " == " << time_act << std::endl;
            if (time_exp != time_act) {
                std::cerr << "Assertion failure" << std::endl;
                exit(1);
            }
        }
    }


    void test() {
        constexpr const char *cases[] = {"small", "medium", "large"};

        for (const char *const case_name: cases) {
            std::stringstream out_act;

            {
                std::ostringstream fnin;
                fnin << "samples/sample_input_" << case_name << ".txt";
                std::ifstream in;
                in.open(fnin.str());
                process_streams(in, out_act);
            }

            std::ostringstream fnout;
            fnout << "samples/sample_output_" << case_name << ".txt";
            std::ifstream out_exp;
            out_exp.exceptions(std::ios::badbit);
            out_exp.open(fnout.str());

            out_act.seekp(0);
            compare(out_exp, out_act);
        }
    }


    void process_std() {
        std::ios::sync_with_stdio(false);  // Critical to fast handling of stdin
        process_streams(std::cin, std::cout);
    }
}


int main(int argc, const char **argv) {
    try {
        if (argc > 1 && argv[1] == "-t"sv)
            test();
        else process_std();
    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
