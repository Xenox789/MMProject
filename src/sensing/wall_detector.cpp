#include "sensing/wall_detector.h"
#include "constants.h"

namespace sensing {

namespace {

inline bool schmitt(bool prev, int value, int on, int off) {
    if (prev)  return value > off;   // stay ON until reading drops below OFF
    else       return value > on;    // turn ON only when reading exceeds ON
}

} // namespace

const WallView &WallDetector::update(const hal::IrReading &r) {
    const int fl60 = r.reflect[hal::IR_FL60];
    const int fl05 = r.reflect[hal::IR_FL05];
    const int fr05 = r.reflect[hal::IR_FR05];
    const int fr60 = r.reflect[hal::IR_FR60];

    const int front_avg = (fl05 + fr05) / 2;

    m_view.front_metric = front_avg;

    m_view.front = schmitt(m_view.front, front_avg,
                           CONFIG::IR_FRONT_WALL_ON,
                           CONFIG::IR_FRONT_WALL_OFF);
    m_view.left  = schmitt(m_view.left,  fl60,
                           CONFIG::IR_SIDE_WALL_ON,
                           CONFIG::IR_SIDE_WALL_OFF);
    m_view.right = schmitt(m_view.right, fr60,
                           CONFIG::IR_SIDE_WALL_ON,
                           CONFIG::IR_SIDE_WALL_OFF);

    // Lateral centring error.
    //   positive  -> too close to LEFT wall  -> steer RIGHT
    //   negative  -> too close to RIGHT wall -> steer LEFT
    const int err_l = fl60 - CONFIG::IR_CENTER_TARGET_LEFT;
    const int err_r = fr60 - CONFIG::IR_CENTER_TARGET_RIGHT;

    if (m_view.left && m_view.right)      m_view.lateral_error = err_l - err_r;
    else if (m_view.left)                 m_view.lateral_error = err_l;
    else if (m_view.right)                m_view.lateral_error = -err_r;
    else                                  m_view.lateral_error = 0;

    return m_view;
}

} // namespace sensing
