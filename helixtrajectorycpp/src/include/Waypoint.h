#pragma once

#include <vector>

#include "InitialGuessPoint.h"
#include "Obstacle.h"
#include "PositionConstraint.h"

namespace helixtrajectory {

    /**
     * @brief A certain state that the robot must have at some sample point of the trajectory.
     * Waypoint includes options to constrain dynamics like position and velocity at the waypoint
     * sample point. The waypoint sample point is defined as the sample point associated with the
     * waypoint. The trajectory segment, or segment, is the ordered collection of sample points
     * that lead up to the waypoint, including the waypoint sample point but not including the
     * previous waypoint sample point in the path if it exists (if this is not the first waypiont).
     * 
     * @author Justin Babilino
     */
    class Waypoint {
    public:
        /**
         * @brief the constraint on position for this waypoint; these constraints are applied
         * only to the sample point associated with the waypoint
         */
        PositionConstraint waypointPositionConstraint;
        /**
         * @brief the constraint on position for this segment; these constraints are applied
         * to the sample points of the segment but are only additionally applied to the waypoint
         * sample point if applySegmentConstraintsToWaypoint is set to true
         */
        PositionConstraint segmentPositionConstraint;
        /**
         * @brief whether or not to apply the constraints specified in the segment constraint set to the
         * sample point at the waypoint in addition to the sample points in the segment; the constraint
         * at this sample will be the union of the segment and waypint constraints.
         */
        bool applySegmentPositionConstraintAtWaypoint;

        /**
         * @brief the number of control intervals in the optimization problem from the previous waypoint to this
         * waypoint
         */
        size_t controlIntervalCount;
        /**
         * @brief the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         */
        std::vector<InitialGuessPoint> initialGuessPoints;

        /**
         * @brief Destroy the Waypoint object
         */
        virtual ~Waypoint() = default;

        /**
         * @brief Check if this waypoint is an initial waypoint. An initial
         * waypoint is a waypoint that has zero control intervals.
         * 
         * @return true if and only if controlIntervalTotal == 0
         */
        bool IsInitialWaypoint() const noexcept;
        /**
         * @brief Check if this waypoint is valid.
         * 
         * @return true if this waypoint is valid, false otherwise
         */
        bool IsValid() const noexcept;
        /**
         * @brief Check if the position state at this waypoint is known. This
         * means that the position and heading of the robot is constrained.
         * 
         * @return true if the position state of the robot is constrained, false otherwise
         */
        bool IsPositionStateKnown() const noexcept;
        /**
         * @brief Check if the velocity state at this waypoint is known. This
         * means that the velocity of the robot is constrained.
         * 
         * @return true if the velocity state of the robot is constrained, false otherwise
         */
        virtual bool IsVelocityStateKnown() const noexcept = 0;
        /**
         * @brief Check if this waypoint is a known state waypoint. A known
         * state waypoint is a waypoint with its state fully constrained and known
         * before generation.
         * 
         * @return true if the position and velocity state of the robot is constrained,
         * false otherwise
         */
        bool IsStateKnown() const noexcept;

        /**
         * @brief Check if this waypoint is a split waypoint. A split waypoint
         * is a waypoint that is either an initial waypoint or known state waypoint.
         * 
         * @return true if and only if this waypoint is an initial waypoint or a known state waypoint
         */
        bool IsSplitWaypoint() const noexcept;

    protected:
        /**
         * @brief Construct a Waypoint with its position constraint settings, control interval count,
         * initial guess points, and obstacles.
         * 
         * @param waypointPositionConstraint the set of constraints on position for this waypoint
         * @param segmentPositionConstraint the set of constraints on position for this segment
         * @param applySegmentConstraintAtWaypoint whether or not to apply the constraint specified in
         * the segment constraint set to the waypoint sample point
         * @param controlIntervalCount the number of control intervals in the optimization problem from the previous waypoint to this
         * waypoint
         * @param initialGuessPoints the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         * @param obstacles the collection of obstacles that the robot must avoid while approaching this waypoint
         */
        Waypoint(const PositionConstraint& waypointPositionConstraint,
                const PositionConstraint& segmentPositionConstraint,
                bool applySegmentConstraintAtWaypoint,
                size_t controlIntervalCount,
                const std::vector<InitialGuessPoint>& initialGuessPoints);
    };
}