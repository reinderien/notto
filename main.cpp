#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numbers>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>


using namespace std::string_view_literals;


namespace {
    constexpr int
        delay = 10,
        speed = 2;
    constexpr double
        dist_min = 1,
        dist_max = 100*std::numbers::sqrt2,
        time_min = dist_min / speed,
        time_max = dist_max / speed;

    class Waypoint {
    public:
        const int x, y, penalty = 0;

        static Waypoint read(std::istream &in) {
            int x, y, penalty;
            in >> x >> y >> penalty;
            return Waypoint(x, y, penalty);
        }

        double time_to(const Waypoint &other) const {
            int dx = x - other.x, dy = y - other.y;
            // std::hypot(dx, dy) makes better use of the library but is much slower
            double distance = sqrt(dx*dx + dy*dy);
            return distance / speed;
        }
    };


    class OptimisedWaypoint {
    public:
        const Waypoint &waypoint;
        const double best_cost = 0;
        const int penalty = 0;

        double cost_from(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            return time + invariant_cost() + delay;
        }

        double invariant_cost() const {
            return best_cost - penalty;
        }

        void emplace(std::multimap<double, OptimisedWaypoint> &into) const {
            into.emplace(invariant_cost(), *this);
        }
    };


    void prune(std::multimap<double, OptimisedWaypoint> &opt_waypoints) {
        double to_exceed = opt_waypoints.cbegin()->first + time_max - time_min;
        auto prune_from = opt_waypoints.lower_bound(to_exceed);
        opt_waypoints.erase(prune_from, opt_waypoints.cend());
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
            assert(time_exp == time_act);
        }
    }


    void test() {
        constexpr const char *cases[] = {"small", "medium", "large"};

        for (const char *const case_name: cases) {
            std::stringstream out_act;

            {
                std::ostringstream fnin;
                fnin << "sample_input_" << case_name << ".txt";
                std::ifstream in;
                in.open(fnin.str());
                process_streams(in, out_act);
            }

            std::ostringstream fnout;
            fnout << "sample_output_" << case_name << ".txt";
            std::ifstream out_exp;
            out_exp.exceptions(std::ios::badbit);
            out_exp.open(fnout.str());

            out_act.seekp(0);
            compare(out_exp, out_act);
        }
    }


    void main() {
        process_streams(std::cin, std::cout);
    }
}


int main(int argc, const char **argv) {
    try {
        if (argc > 1 && argv[1] == "-t"sv)
            test();
        else main();
    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
