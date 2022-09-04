#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>


namespace {
    constexpr int
        delay = 10,
        speed = 2;
    constexpr double
        dist_min = 1,
        dist_max = 100*M_SQRT2,
        time_min = dist_min / speed,
        time_max = dist_max / speed;

    class Waypoint {
    public:
        const int x, y, penalty = 0;

        Waypoint(int x, int y) : x(x), y(y) { }
        Waypoint(int x, int y, int penalty) : x(x), y(y), penalty(penalty) { }

        static Waypoint read(std::istream &in) {
            int x, y, penalty;
            in >> x >> y >> penalty;
            return Waypoint(x, y, penalty);
        }

        double time_to(const Waypoint &other) const {
            int dx = x - other.x, dy = y - other.y;
            double distance = sqrt(dx*dx + dy*dy);
            return distance / speed;
        }
    };


    class OptimisedWaypoint {
    public:
        const Waypoint &waypoint;
        const double best_cost = 0, accrued_penalty = 0;

        OptimisedWaypoint(const Waypoint &visited): waypoint(visited) { }

        OptimisedWaypoint(const Waypoint &visited, double best_cost):
            waypoint(visited), best_cost(best_cost) { }

        OptimisedWaypoint(const OptimisedWaypoint &copy, double penalty_delta):
            waypoint(copy.waypoint), best_cost(copy.best_cost),
            accrued_penalty(copy.accrued_penalty + penalty_delta) { }

        double cost_for(const Waypoint &visited) const {
            double time = visited.time_to(waypoint);
            return time + best_cost + accrued_penalty + delay;
        }

        double invariant_cost() const {
            return accrued_penalty + best_cost;
        }

        bool operator<(const OptimisedWaypoint &other) const {
            return invariant_cost() < other.invariant_cost();
        }
    };


    void prune(std::multiset<OptimisedWaypoint> &opt_waypoints) {
        double to_exceed = opt_waypoints.cbegin()->invariant_cost() + time_max - time_min;

        for (;;) {
            auto last = std::prev(opt_waypoints.cend());
            if (last->invariant_cost() > to_exceed)
                opt_waypoints.erase(last);
            else break;
        }
    }


    double get_best_cost(
        const Waypoint &visited,
        std::multiset<OptimisedWaypoint> &opt_waypoints
    ) {
        double best_cost = std::numeric_limits<double>::max();

        for (auto skipto = opt_waypoints.cbegin(); skipto != opt_waypoints.cend(); skipto++) {
            double cost = skipto->cost_for(visited);
            if (best_cost > cost) best_cost = cost;
        }

        return best_cost;
    }


    double solve(const std::vector<Waypoint> &waypoints) {
        std::multiset<OptimisedWaypoint> opt_waypoints_a, opt_waypoints_b;
        std::multiset<OptimisedWaypoint>
            *opt_waypoints_on = &opt_waypoints_a,
            *opt_waypoints_off = &opt_waypoints_b;
        opt_waypoints_on->emplace(waypoints.back());

        auto end = std::prev(waypoints.crend());
        for (auto visited = std::next(waypoints.crbegin());; visited++) {
            double best_cost = get_best_cost(*visited, *opt_waypoints_on);
            if (visited == end)
                return best_cost;

            for (auto w = opt_waypoints_on->cbegin(); w != opt_waypoints_on->cend(); w++) {
                opt_waypoints_off->emplace(*w, visited->penalty);
            }
            std::swap(opt_waypoints_on, opt_waypoints_off);
            opt_waypoints_off->clear();

            opt_waypoints_on->emplace(*visited, best_cost);

            prune(*opt_waypoints_on);
        }
    }


    void process_streams(std::istream &in, std::ostream &out) {
        constexpr std::ios::iostate mask = std::ios::failbit | std::ios::badbit;
        in.exceptions(mask);
        out.exceptions(mask);

        for (;;) {
            int n;
            in >> n;
            if (n == 0) break;

            std::vector<Waypoint> waypoints;
            waypoints.reserve(n+2);
            waypoints.emplace_back(0, 0);
            for (int i = 0; i < n; i++)
                waypoints.push_back(Waypoint::read(in));
            waypoints.emplace_back(100, 100);

            out << std::fixed << std::setprecision(3) << solve(waypoints) << std::endl;
        }
    }


    void compare(std::istream &out_exp, std::istream &out_act) {
        for (;;) {
            std::string time_exp, time_act;
            if (!std::getline(out_exp, time_exp)) {
                if (out_exp.eof()) break;
                throw std::ios::failure("getline");
            }
            std::cout << time_exp << std::endl;
            out_act >> time_act;
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
        if (argc > 1 && std::string(argv[1]) == "-t")
            test();
        else main();
    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
