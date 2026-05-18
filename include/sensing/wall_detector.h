// ============================================================================
//  sensing/wall_detector.h  --  Translate raw IR samples to WallView.
//
//  Rules:
//      * front:  true if (FL_05 + FR_05)/2 >= IR_FRONT_WALL_ON,
//                false  when it falls below IR_FRONT_WALL_OFF.
//        (Average of the two near-forward sensors -- robust to one side
//        being slightly mis-aimed.)
//      * left:   IR_FL60 reading crosses IR_SIDE_WALL_ON/OFF.
//      * right:  IR_FR60 reading crosses IR_SIDE_WALL_ON/OFF.
//      * lateral_error: (FL60 - CENTER_TARGET_LEFT) - (FR60 - CENTER_TARGET_RIGHT)
//        when both side walls are seen; FL60-CENTER alone if only left;
//        -(FR60-CENTER) if only right; 0 if neither (open).
//
//  All state is encapsulated -- no globals.  Detector instances are cheap;
//  the system has exactly one in the sensing task.
// ============================================================================

#pragma once

#include "sensing/wall_view.h"
#include "hal/ir_array.h"

namespace sensing {

class WallDetector {
public:
    // Feed a fresh IR sample; returns the latched WallView.
    const WallView &update(const hal::IrReading &r);

    const WallView &latest() const { return m_view; }

private:
    WallView m_view;
};

} // namespace sensing
