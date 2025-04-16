#ifndef PATH_MODES_HPP
#define PATH_MODES_HPP

namespace auv_core_helper {

enum PathMode { // Changed from `enum class` to `enum`
    Serpentine2D = 1,
    Serpentine3D = 2,
    Helix3D = 3
};

} // namespace auv_core_helper

#endif // PATH_MODES_HPP
