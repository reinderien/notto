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
    constexpr int speed = 2, delay = 10;
    constexpr double
        dist_min = 1,
        dist_max = 100*M_SQRT2,
        time_min = dist_min / speed,
        time_max = dist_max / speed,
        stored_cost_min = delay + time_min;

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

            /*
            This innermost loop produces an overall algorithm that is O(n^2). For random data, the necessary search
            space starting at the current visited node is far shorter than the sequence to the end - only about the
            first 15 or so best_costs need to be checked, because at scales longer than that the best_costs increase
            monotonically. If such a guarantee can be made, this algorithm is reduced to an O(n)-amortised complexity in
            time, and if a bounded queue is used for waypoints, then an O(1)-amortised complexity in space.

            Whereas a 15-node-abridged search does pass the provided test cases (very quickly), it is possible to
            construct an input test case with spatially adjacent nodes and maximal skip penalty where the best_costs
            search space tends to decrease rather than increase, and abridgement is not feasible. To accommodate cases
            like this, leave the O(n^2) solution in place.
            */
            for (; skipto != end; skipto++) {
                double time = time_to(*skipto),
                       cost = time + penalties + skipto->best_cost;
                if (best_cost > cost) best_cost = cost;
                penalties += skipto->penalty;

                // We could further constrain this to penalties + time_min + stored_cost_min,
                // but the calculation costs more than it saves
                double min_next_cost = penalties;
                if (best_cost <= min_next_cost)
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
