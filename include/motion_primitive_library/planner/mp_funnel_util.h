/**
 * @file mp_funnel_util.h
 * @brief motion planning cloud util
 */
#include <motion_primitive_library/planner/env_funnel.h>
#include <motion_primitive_library/planner/mp_base_util.h>

/**
 * @brief Motion primitive planner using point cloud
 */
class MPFunnelUtil : public MPBaseUtil<3>
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPFunnelUtil(bool verbose);
    ///Set map util
    void setMap(const vec_Vec3f& obs, const Vec3f& ori, const Vec3f& dim,
        decimal_t kp, decimal_t kv, decimal_t v);
};
