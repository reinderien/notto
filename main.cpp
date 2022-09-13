#include <algorithm>
#include <cassert>
#include <charconv>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>


using namespace std::string_view_literals;


namespace {
    constexpr int
        delay = 10,  // seconds
        speed = 2,   // metres per second
        edge = 100;  // metres

    // These are theoretical bounds; we get narrower than this
    constexpr double
        dist_min = 1,
        dist_max = edge*std::numbers::sqrt2,
        time_min = dist_min / speed,
        time_max = dist_max / speed;

    constexpr std::errc success = std::errc();


    double time_to(int dx, int dy) {
        // std::hypot(dx, dy) makes better use of the library but is much slower

        assert(dx >= -edge); assert(dx <= edge);
        assert(dy >= -edge); assert(dy <= edge);

        // We cannot assert that this time is within time_min and time_max, due to the case where time_min() is called
        // on the endpoints
        return sqrt(dx*dx + dy*dy) / speed;
    }


    int coord_min(int x) {
        return std::max(1, std::min(edge-x, x));
    }

    int coord_max(int x) {
        return std::max(edge-x, x);
    }


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


    class OptimisedWaypoint {
    public:
        const Waypoint waypoint;
        const double best_cost, invariant_cost, cost_min;
        const int penalty;

        OptimisedWaypoint(const Waypoint &waypoint, double best_cost = 0):
            waypoint(waypoint), best_cost(best_cost), invariant_cost(best_cost - waypoint.penalty + delay),
            cost_min(waypoint.time_min() + invariant_cost), penalty(waypoint.penalty) { }

        double cost_from(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            assert(!std::isnan(time));
            // We cannot do this because the provided sample_input_large.txt violates the uniqueness specification for
            // waypoints at (4, 2)
            // assert(time >= time_min);
            assert(time <= time_max);
            return time + invariant_cost;
        }

        double cost_max() const {
            return waypoint.time_max() + invariant_cost;
        }

        bool emplace(std::multimap<double, OptimisedWaypoint> &into) const {
            return into.cbegin() == into.emplace(cost_min, *this);
        }

        void output(std::ostream &out) const {
            out << waypoint
                << " best_cost=" << best_cost
                << " inv_cost=" << invariant_cost
                << " cost_min=" << cost_min
                << " penalty=" << penalty;
        }

        bool is_sane() const {
            return waypoint.is_sane();
        }
    };

    std::ostream &operator<<(std::ostream &out, const OptimisedWaypoint &ow) {
        ow.output(out);
        return out;
    }


    void prune(std::multimap<double, OptimisedWaypoint> &opt_waypoints, double to_exceed) {
        auto prune_from = opt_waypoints.upper_bound(to_exceed);
        assert(prune_from != opt_waypoints.cbegin());
        opt_waypoints.erase(prune_from, opt_waypoints.cend());
    }


    double get_best_cost(
        const Waypoint &visited,
        const std::multimap<double, OptimisedWaypoint> &opt_waypoints
    ) {
        double best_cost = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skipfrom: opt_waypoints | std::views::values) {
            assert(skipfrom.is_sane());
            best_cost = std::min(best_cost, skipfrom.cost_from(visited));
        }

        assert(best_cost < std::numeric_limits<double>::max());
        return best_cost;
    }


    double solve(WaypointReader &reader, int n) {
        std::multimap<double, OptimisedWaypoint> opt_waypoints;
        double acceptable_cost = std::numeric_limits<double>::max();
        int total_penalty = 0;

        const OptimisedWaypoint head(Waypoint(0, 0));
        head.emplace(opt_waypoints);

        for (int i = 0; i < n; ++i) {
            Waypoint visited = reader.get_next();
            assert(visited.is_sane());
            total_penalty += visited.penalty;

            double best_cost = get_best_cost(visited, opt_waypoints);
            OptimisedWaypoint new_opt(visited, best_cost);
            assert(new_opt.is_sane());

            if (new_opt.cost_min <= acceptable_cost && new_opt.emplace(opt_waypoints)) {
                acceptable_cost = new_opt.cost_max();
                prune(opt_waypoints, acceptable_cost);
            }
        }

        constexpr Waypoint visited(edge, edge);
        double best_cost = get_best_cost(visited, opt_waypoints);
        return best_cost + total_penalty;
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
        std::ios::sync_with_stdio(false);
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
