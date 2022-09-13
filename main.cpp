#include <cassert>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <ranges>


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


    double time_to(int dx, int dy) {
        assert(-edge <= dx); assert(dx <= edge);
        assert(-edge <= dy); assert(dy <= edge);

        // std::hypot(dx, dy) makes better use of the library but is much slower
        double time = sqrt(dx*dx + dy*dy) / speed;
        assert(!std::isnan(time));
        assert(time_min <= time); assert(time <= time_max);

        return time;
    }


    int coord_min(int x) {
        return std::min(edge-x, x);
    }

    int coord_max(int x) {
        return std::max(edge-x, x);
    }


    // Direct representation of waypoints parsed from the input
    class Waypoint {
    public:
        const int x, y, penalty = 0;

        double time_to(const Waypoint &other) const {
            return ::time_to(x - other.x, y - other.y);
        }

        double time_min() const {
            return ::time_to(coord_min(x), coord_min(y));
        }

        double time_max() const {
            return ::time_to(coord_max(x), coord_max(y));
        }

        void output(std::ostream &out) const {
            out << "x=" << x << " y=" << y << " penalty=" << penalty;
        }

        bool is_sane() const {
            return x >= 0 && x <= edge &&
                   y >= 0 && y <= edge;
        }
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

    public:
        WaypointReader(const std::string &body_str):
            body_mem(body_str), body_view(body_mem) {
        }

        static WaypointReader from_stream(std::istream &in) {
            std::stringstream incopy;
            incopy << in.rdbuf();
            return WaypointReader(incopy.str());
        }

        int get_case_size() {
            size_t next_pos = body_view.find('\n', pos),
                   substr_len = next_pos - pos;
            std::string_view line = body_view.substr(pos, substr_len);
            const char *line_start = line.data(),
                       *line_end = line_start + substr_len;
            pos = next_pos + 1;

            int size;
            std::from_chars_result r = std::from_chars(line_start, line_end, size);
            if (r.ec != success || r.ptr != line_end)
                throw parse_error;

            return size;
        }

        Waypoint get_next() {
            size_t next_pos = body_view.find('\n', pos),
                   substr_len = next_pos - pos;
            std::string_view line = body_view.substr(pos, substr_len);
            const char *line_start = line.data(),
                       *line_end = line_start + substr_len;
            pos = next_pos+1;

            int x, y, penalty;
            std::from_chars_result r = std::from_chars(line_start, line_end, x);
            if (r.ec != success)
                throw parse_error;

            r = std::from_chars(r.ptr+1, line_end, y);
            if (r.ec != success)
                throw parse_error;

            r = std::from_chars(r.ptr+1, line_end, penalty);
            if (r.ec != success || r.ptr != line_end)
                throw parse_error;

            return Waypoint(x, y, penalty);
        }
    };

    const std::invalid_argument WaypointReader::parse_error("Invalid input line");


    // A sidekick to Waypoint that includes optimiser data. Only one "visited" Waypoint is held
    // in memory at a time, but a small handful of OptimisedWaypoints are held in a working map.
    class OptimisedWaypoint {
    public:
        const Waypoint waypoint;
        const double cost_best,       // Cost of the optimal path from the beginning all the way here
                     cost_invariant,  // Sum of invariant costs incurred by skipping from this waypoint
                     cost_min;        // Lowest possible cost incurred by skipping from this waypoint to anywhere

        OptimisedWaypoint(const Waypoint &waypoint, double cost_best = 0):
            waypoint(waypoint), cost_best(cost_best), cost_invariant(cost_best - waypoint.penalty + delay),
            cost_min(waypoint.time_min() + cost_invariant) { }

        double cost_to(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            return time + cost_invariant;
        }

        double cost_max() const {
            return waypoint.time_max() + cost_invariant;
        }

        // Return true if we emplace at the beginning of the map,
        // that is, if we become the lowest-minimum-cost waypoint
        bool emplace(std::multimap<double, OptimisedWaypoint> &into) const {
            return into.cbegin() == into.emplace(cost_min, *this);
        }

        void output(std::ostream &out) const {
            out << waypoint
                << " cost_best=" << cost_best
                << " cost_inv=" << cost_invariant
                << " cost_min=" << cost_min;
        }

        bool is_sane() const {
            return waypoint.is_sane();
        }
    };

    std::ostream &operator<<(std::ostream &out, const OptimisedWaypoint &ow) {
        ow.output(out);
        return out;
    }


    // Erase all map waypoints whose minimum cost is greater than to_exceed. to_exceed is the maximum cost of the map's
    // front waypoint, having the lowest minimum cost of any optimised waypoint.
    void prune(std::multimap<double, OptimisedWaypoint> &opt_waypoints, double to_exceed) {
        auto prune_from = opt_waypoints.upper_bound(to_exceed);
        assert(prune_from != opt_waypoints.cbegin());
        opt_waypoints.erase(prune_from, opt_waypoints.cend());
    }


    double get_best_cost(
        const Waypoint &visited,
        const std::multimap<double, OptimisedWaypoint> &opt_waypoints
    ) {
        double cost_best = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skipfrom: opt_waypoints | std::views::values)
            cost_best = std::min(cost_best, skipfrom.cost_to(visited));

        assert(cost_best < std::numeric_limits<double>::max());
        return cost_best;
    }


    double solve(WaypointReader &reader, int n) {
        // Map of optimised waypoints in increasing order of their minimum possible skip-from cost
        std::multimap<double, OptimisedWaypoint> opt_waypoints;

        // The maximum acceptable cost, set as the maximum possible cost of the lowest-minimum-cost waypoint.
        // Any waypoints costing more than this are discarded.
        double acceptable_cost = std::numeric_limits<double>::max();

        int total_penalty = 0;

        const OptimisedWaypoint head(Waypoint(0, 0));
        head.emplace(opt_waypoints);

        for (int i = 0; i < n; ++i) {
            Waypoint visited = reader.get_next();
            assert(visited.is_sane());
            total_penalty += visited.penalty;

            double cost_best = get_best_cost(visited, opt_waypoints);
            OptimisedWaypoint new_opt(visited, cost_best);
            assert(new_opt.is_sane());

            if (new_opt.cost_min <= acceptable_cost && new_opt.emplace(opt_waypoints)) {
                acceptable_cost = new_opt.cost_max();
                // Only prune if the new waypoint has been accepted and has become the lowest-minimum-cost waypoint.
                // Otherwise, the cost bounds will not have changed.
                prune(opt_waypoints, acceptable_cost);
            }
        }

        constexpr Waypoint tail(edge, edge);
        double cost_best = get_best_cost(tail, opt_waypoints);

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
