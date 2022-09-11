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
        delay = 10,
        speed = 2,
        edge = 100;

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

        return sqrt(dx*dx + dy*dy) / speed;
    }


    class Waypoint {
    public:
        const int x, y, penalty = 0;

        double time_to(const Waypoint &other) const {
            return ::time_to(x - other.x, y - other.y);
        }

        double time_min() const {
            return ::time_to(std::min(x, edge-x), std::min(y, edge-y));
        }

        double time_max() const {
            return ::time_to(std::max(x, edge-x), std::max(y, edge-y));
        }

        void output(std::ostream &out) const {
            out << "x=" << x
                << " y=" << y
                << " penalty=" << penalty;
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
        const std::string_view body;
        long pos;
        int x, y, penalty, case_count = 0;

    public:
        enum NextState {
            has_next, case_end, err
        };

        WaypointReader(const std::string &body) : body_mem(body), body(body_mem), pos(body_mem.size()-1) {
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
            size_t next_pos = body.rfind('\n', pos-1),
                   substr_len = pos - next_pos - 1;
            pos = next_pos;
            std::string_view line = body.substr(next_pos+1, substr_len);
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
            return time + invariant_cost;
        }

        double cost_max() const {
            return waypoint.time_max() + invariant_cost;
        }

        static void emplace(const OptimisedWaypoint &ow, std::multimap<double, OptimisedWaypoint> &into) {
            into.emplace(ow.cost_min, ow);
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
        opt_waypoints.erase(prune_from, opt_waypoints.cend());
        assert(opt_waypoints.size() > 0);
    }


    double get_best_cost(
        const Waypoint &visited,
        const std::multimap<double, OptimisedWaypoint> &opt_waypoints
    ) {
        double best_cost = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skipto: opt_waypoints | std::views::values) {
            assert(skipto.is_sane());
            double cost = skipto.cost_from(visited);
            if (best_cost > cost) best_cost = cost;
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
            OptimisedWaypoint::emplace(tail, opt_waypoints);
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
            OptimisedWaypoint::emplace(new_opt, opt_waypoints);
            assert(new_opt.is_sane());
            prune(opt_waypoints);
        }

        double finish() {
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

        for (auto t = times.crbegin(); t != times.crend(); t++)
            out << *t << '\n';
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
