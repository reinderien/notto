#include <algorithm>
#include <cassert>
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
        speed = 2;

    // These are theoretical bounds; we get narrower than this
    constexpr double
        dist_min = 1,
        time_min = dist_min / speed,
        dist_max = 100*std::numbers::sqrt2,
        time_max = dist_max / speed;


    double time_to(int dx, int dy) {
        // std::hypot(dx, dy) makes better use of the library but is much slower
        return sqrt(dx*dx + dy*dy) / speed;
    }


    class Waypoint {
    public:
        const int x, y, penalty;
        const double time_min;

        Waypoint(int x, int y, int penalty = 0):
            x(x), y(y), penalty(penalty),
            time_min(::time_to(std::min(x, 100-x), std::min(y, 100-y))) { }

        static Waypoint read(std::istream &in) {
            // This is a bottleneck. The following code is a replacement for the typical strategy of
            //    in >> x >> y >> penalty;
            std::string line;
            std::getline(in, line);

            const std::string::size_type
                iy = 1+line.find(' '),
                ip = 1+line.find(' ', iy);
            const int x = std::stoi(line),
                      y = std::stoi(line.substr(iy)),
                penalty = std::stoi(line.substr(ip));
            return Waypoint(x, y, penalty);
        }

        double time_to(const Waypoint &other) const {
            return ::time_to(x - other.x, y - other.y);
        }

        double time_max() const {
            return ::time_to(std::max(x, 100 - x), std::max(y, 100 - y));
        }

        void output(std::ostream &out) const {
            out << "x=" << x
                << " y=" << y
                << " penalty=" << penalty
                << " time_min=" << time_min;
        }
    };

    std::ostream &operator<<(std::ostream &out, const Waypoint &w) {
        w.output(out);
        return out;
    }


    class OptimisedWaypoint {
    public:
        const Waypoint &waypoint;
        const double best_cost, invariant_cost, cost_min;
        const int penalty;

        OptimisedWaypoint(const Waypoint &waypoint, double best_cost = 0, int penalty = 0):
            waypoint(waypoint), best_cost(best_cost), invariant_cost(best_cost - penalty + delay),
            cost_min(waypoint.time_min + invariant_cost), penalty(penalty) { }

        double cost_from(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
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
    };

    std::ostream &operator<<(std::ostream &out, const OptimisedWaypoint &ow) {
        ow.output(out);
        return out;
    }


    void prune(std::multimap<double, OptimisedWaypoint> &opt_waypoints) {
        const OptimisedWaypoint &front = opt_waypoints.cbegin()->second;
        const double to_exceed = front.cost_max();
        auto prune_from = opt_waypoints.upper_bound(to_exceed);
        assert(prune_from != opt_waypoints.cbegin());

        opt_waypoints.erase(prune_from, opt_waypoints.cend());
        assert(opt_waypoints.size() > 0);
    }


    double get_best_cost(
        const Waypoint &visited,
        const std::multimap<double, OptimisedWaypoint> &opt_waypoints
    ) {
        double best_cost = std::numeric_limits<double>::max();

        for (const OptimisedWaypoint &skipto: opt_waypoints | std::views::values) {
            double cost = skipto.cost_from(visited);
            if (best_cost > cost) best_cost = cost;
        }

        assert(best_cost < std::numeric_limits<double>::max());
        return best_cost;
    }


    double solve(const std::vector<Waypoint> &waypoints) {
        std::multimap<double, OptimisedWaypoint> opt_waypoints;
        OptimisedWaypoint(waypoints.back()).emplace(opt_waypoints);
        const auto end = std::prev(waypoints.crend());

        int total_penalty = 0;

        for (auto visited = std::next(waypoints.crbegin());; ++visited) {
            total_penalty += visited->penalty;
            double best_cost = get_best_cost(*visited, opt_waypoints);
            if (visited == end)
                return best_cost + total_penalty;

            OptimisedWaypoint(*visited, best_cost, visited->penalty).emplace(opt_waypoints);

            prune(opt_waypoints);
        }
    }


    void process_streams(std::istream &in, std::ostream &out) {
        constexpr std::ios::iostate mask = std::ios::failbit | std::ios::badbit;
        in.exceptions(mask);
        out.exceptions(mask);
        out << std::fixed << std::setprecision(3);

        for (;;) {
            int n;
            in >> n;
            if (n == 0) break;
            std::string empty;
            std::getline(in, empty);

            std::vector<Waypoint> waypoints;
            waypoints.reserve(n+2);
            waypoints.emplace_back(0, 0);
            for (int i = 0; i < n; ++i)
                waypoints.push_back(Waypoint::read(in));
            waypoints.emplace_back(100, 100);

            out << solve(waypoints) << std::endl;
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
