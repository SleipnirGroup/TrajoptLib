#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "solution/HolonomicSolution.h"

namespace helixtrajectory {

struct SwerveSolution : HolonomicSolution {

    std::vector<std::vector<double>> moduleFX;
    std::vector<std::vector<double>> moduleFY;
};
}

template<>
struct fmt::formatter<helixtrajectory::SwerveSolution> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::SwerveSolution& swerveSolution, FormatContext& ctx) {
        std::string tableEntries;
        for (std::size_t index = 1; index < swerveSolution.x.size(); index++) {
            tableEntries += fmt::format(
                "  {2:>{0}.{1}f} ║ {3:>{0}.{1}f} ║ {4:>{0}.{1}f} ║ {5:>{0}.{1}f}  \n",
                    12,
                    6,
                    swerveSolution.dt[index - 1],
                    swerveSolution.x[index],
                    swerveSolution.y[index],
                    swerveSolution.theta[index]);
        }
        return fmt::format_to(ctx.out(),
            "  {2:<{1}} ║ {3:<{1}} ║ {4:<{1}} ║ {5:<{1}}  \n"
            "══{0:═^{1}}═╬═{0:═^{1}}═╬═{0:═^{1}}═╬═{0:═^{1}}══\n"
            "  {0:>{1}} ║ {12:>{1}.6f} ║ {13:>{1}.6f} ║ {14:>{1}.6f}  \n"
            "{21}",
                "",                       // 0
                12,                        // 1
                "dt",                     // 2
                "x",                      // 3
                "y",                      // 4
                "θ",                      // 5
                "vₓ",                     // 6
                "vᵧ",                     // 7
                "ω",                      // 8
                "aₓ",                     // 9
                "aᵧ",                     // 10
                "α",                      // 11
                swerveSolution.x[0],      // 12
                swerveSolution.y[0],      // 13
                swerveSolution.theta[0],  // 14
                swerveSolution.vx[0],     // 15
                swerveSolution.vy[0],     // 16
                swerveSolution.omega[0],  // 17
                swerveSolution.ax[0],     // 18
                swerveSolution.ay[0],     // 19
                swerveSolution.alpha[0],   // 20
                tableEntries);
    }
};
/*
╔ ╦ ╗
║ ║ ║
╠ ╬ ╣
╚ ╩ ╝
*/