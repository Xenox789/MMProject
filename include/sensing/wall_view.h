// ============================================================================
//  sensing/wall_view.h  --  Plain-old-data view of "what walls are around me?"
//
//  Decoupled from the IR hardware on purpose: the maze solver and the
//  motion controller both consume this struct, and neither should care how
//  the walls were detected.
// ============================================================================

#pragma once

#include <cstdint>

namespace sensing {

struct WallView {
    bool front = false;   // wall directly ahead, within "stop" distance
    bool left  = false;   // wall on the immediate left
    bool right = false;   // wall on the immediate right

    // Continuous lateral error in raw IR units.  Positive = mouse is too
    // close to the LEFT wall (so steer right); negative = too close to the
    // right wall.  Zero if neither side wall is seen (open corridor).
    int lateral_error = 0;

    // Continuous distance-to-front-wall metric (raw IR units).  Higher means
    // closer.  When no front wall is detected this falls below the OFF
    // threshold and front = false.
    int front_metric = 0;
};

} // namespace sensing


// ----------------------------------------------------------------------------
//  Compatibility alias for the existing maze module.
//  The old code uses ::WallInfo with members {front, left, right}.  We keep
//  that type available so include/maze.h continues to compile unchanged.
// ----------------------------------------------------------------------------

struct WallInfo {
    bool front = false;
    bool left  = false;
    bool right = false;

    WallInfo() = default;
    WallInfo(const sensing::WallView &v)
        : front(v.front), left(v.left), right(v.right) {}
};
