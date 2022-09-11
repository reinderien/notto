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
        time_min = dist_min / speed,
        dist_max = edge*std::numbers::sqrt2,
        time_max = dist_max / speed;

    constexpr std::errc success = std::errc();

    double time_to(int dx, int dy) {
        // std::hypot(dx, dy) makes better use of the library but is much slower

        assert(dx >= -100); assert(dx <=  100);
        assert(dy >= -100); assert(dy <=  100);

        // We cannot assert that this time is within time_min and time_max, due to the case where time_min() is called
        // on the endpoints
        return sqrt(dx*dx + dy*dy) / speed;
    }


    int coord_min(int x) {
        return std::max(1, std::min(x, edge-x));
    }

    int coord_max(int x) {
        return std::max(x, edge-x);
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
        const std::string body_mem;
        const std::string_view body_view;
        long pos;
        int x, y, penalty, case_count = 0;

    public:
        enum NextState {
            has_next, case_end, err
        };

        WaypointReader(const std::string &body_str):
            body_mem(body_str), body_view(body_mem), pos(body_view.size()-1) {
            NextState expected = advance_state();
            assert(expected == case_end);
        }

        static WaypointReader from_stream(std::istream &in) {
            std::stringstream incopy;
            incopy << in.rdbuf();
            return WaypointReader(incopy.str());
        }

        bool stream_end() const {
            return pos < 0;
        }

        NextState advance_state() {
            size_t next_pos = body_view.rfind('\n', pos-1),
                   substr_len = pos - next_pos - 1;
            pos = next_pos;
            std::string_view line = body_view.substr(next_pos+1, substr_len);
            const char *line_start = line.data(),
                       *line_end = line_start + substr_len;

            std::from_chars_result r = std::from_chars(line_start, line_end, x);
            if (r.ec != success) return err;

            if (r.ptr >= line_end) {
                if (x == case_count) return case_end;
                return err;
            }

            r = std::from_chars(r.ptr+1, line_end, y);
            if (r.ec != success) return err;

            r = std::from_chars(r.ptr+1, line_end, penalty);
            if (r.ec != success) return err;

            return has_next;
        }

        Waypoint get_next() const {
            return Waypoint(x, y, penalty);
        }
    };


    class OptimisedWaypoint {
    public:
        const Waypoint waypoint;
        const double best_cost, invariant_cost, cost_min;
        const int penalty;

        OptimisedWaypoint(const Waypoint &waypoint, double best_cost = 0, int penalty = 0):
            waypoint(waypoint), best_cost(best_cost), invariant_cost(best_cost - penalty + delay),
            cost_min(waypoint.time_min() + invariant_cost), penalty(penalty) { }

        double cost_from(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            assert(!std::isnan(time));
            assert(time >= time_min && time <= time_max);
            return time + invariant_cost;
        }

        double cost_max() const {
            return waypoint.time_max() + invariant_cost;
        }

        void emplace(std::multimap<double, OptimisedWaypoint> &into) const {
            into.emplace(cost_min, *this);
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


    void prune(std::multimap<double, OptimisedWaypoint> &opt_waypoints) {
        const double to_exceed = opt_waypoints.cbegin()->second.cost_max();
        auto prune_from = opt_waypoints.upper_bound(to_exceed);
        assert(prune_from != opt_waypoints.cbegin());
        opt_waypoints.erase(prune_from, opt_waypoints.cend());
    }


    double get_best_cost(
        const Waypoint &visited,
        const std::multimap<double, OptimisedWaypoint> &opt_waypoints
    ) {
        double best_cost = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skipto: opt_waypoints | std::views::values) {
            assert(skipto.is_sane());
            best_cost = std::min(best_cost, skipto.cost_from(visited));
        }

        assert(best_cost < std::numeric_limits<double>::max());
        return best_cost;
    }


    class Solver {
    private:
        std::multimap<double, OptimisedWaypoint> opt_waypoints;
        int total_penalty = 0;
        const OptimisedWaypoint tail = OptimisedWaypoint(Waypoint(edge, edge));

    public:
        Solver() {
            tail.emplace(opt_waypoints);
            assert(is_sane());
        }

        bool is_sane() const {
            for (auto pair: opt_waypoints)
                if (!pair.second.is_sane()) return false;
            return true;
        }

        void feed(const Waypoint &visited) {
            assert(visited.is_sane());
            assert(is_sane());

            total_penalty += visited.penalty;

            double best_cost = get_best_cost(visited, opt_waypoints);

            OptimisedWaypoint new_opt(visited, best_cost, visited.penalty);
            new_opt.emplace(opt_waypoints);
            assert(new_opt.is_sane());
            prune(opt_waypoints);
        }

        double finish() const {
            constexpr Waypoint visited(0, 0);
            double best_cost = get_best_cost(visited, opt_waypoints);
            return best_cost + total_penalty;
        }
    };


    void process_streams(std::istream &in, std::ostream &out) {
        constexpr std::ios::iostate mask = std::ios::failbit | std::ios::badbit;
        in.exceptions(mask);
        out.exceptions(mask);
        out << std::fixed << std::setprecision(3);

        WaypointReader reader = WaypointReader::from_stream(in);

        std::vector<double> times;

        while (!reader.stream_end()) {
            Solver solver;

            while (reader.advance_state() == WaypointReader::has_next) {
                assert(solver.is_sane());
                solver.feed(reader.get_next());
            }
            times.push_back(solver.finish());
        }

        for (auto time = times.crbegin(); time != times.crend(); time++)
            out << *time << '\n';
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
