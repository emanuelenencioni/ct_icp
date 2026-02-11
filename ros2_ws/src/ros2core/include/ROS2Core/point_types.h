#ifndef ROS2CORE_POINT_TYPES_H
#define ROS2CORE_POINT_TYPES_H

#include <cstdint>
#include <cstring>

namespace slam {

    // Plain C++ point structures (no PCL dependency)
    // These are used for internal conversions between slam types and ROS2 messages

    /* Simple 3D point with timestamp */
    struct XYZTPoint {
        float x, y, z;
        uint32_t timestamp;  // in milliseconds
    };

    /* Extended point with intensity and ring */
    struct LidarPoint {
        float x, y, z;
        float intensity;
        uint16_t ring;
        uint32_t timestamp;  // in milliseconds
    };

} // namespace slam

#endif // ROS2CORE_POINT_TYPES_H
