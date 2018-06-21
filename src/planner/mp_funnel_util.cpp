#include <motion_primitive_library/planner/mp_funnel_util.h>

using namespace MPL;

MPFunnelUtil::MPFunnelUtil(bool verbose) : MPBaseUtil<3>::MPBaseUtil() {
  planner_verbose_ = verbose;
  if (planner_verbose_)
    printf(ANSI_COLOR_CYAN
           "[MPFunnelUtil] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void MPFunnelUtil::setMap(const vec_Vec3f &obs, const Vec3f &ori,
                          const Vec3f &dim, decimal_t kp, decimal_t kv,
                          decimal_t v) {
  ENV_.reset(new MPL::env_funnel(obs, ori, dim, kp, kv, v));
}

