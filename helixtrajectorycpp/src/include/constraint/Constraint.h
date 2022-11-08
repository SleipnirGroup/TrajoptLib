#pragma once

#include <variant>

#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"

namespace helixtrajectory {

    using Constraint = std::variant<TranslationConstraint, PoseConstraint, ObstacleConstraint>;
}