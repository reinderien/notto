#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>


namespace {
    constexpr double speed = 2, delay = 10;

    class Waypoint {
    private:
        double best_cost;

    public:
        const int x, y, penalty;

        Waypoint(int x, int y) : x(x), y(y), penalty(0) { }
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

        void set_best_cost(
            std::vector<Waypoint>::const_iterator skipto,
            const std::vector<Waypoint>::const_iterator &end
        ) {
            double best_cost = std::numeric_limits<double>::max(),
                   penalties = delay;

            for (; skipto != end; skipto++) {
                double time = time_to(*skipto),
                       cost = time + penalties + skipto->best_cost;
                if (best_cost > cost) best_cost = cost;
                penalties += skipto->penalty;

                // If the best cost is less than the current penalty sum, we'll never get better
                if (penalties > best_cost)
                    break;
            }

            this->best_cost = best_cost;
        }

        double get_best_cost() const { return best_cost; }
    };


    double solve(std::vector<Waypoint> &waypoints) {
        auto visited = waypoints.rbegin();
        for (visited++; visited != waypoints.rend(); visited++) {
            visited->set_best_cost(visited.base(), waypoints.end());
        }
        return waypoints.begin()->get_best_cost();
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
