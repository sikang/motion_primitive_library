/**
 * @file mp_map_util.h
 * @brief motion primitive map util
 */
#include <planner/env_map.h>
#include <planner/mp_base_util.h>

using linkedHashMap = std::unordered_map<int, std::vector<std::pair<MPL::Key, int>>>;
/**
 * @brief Motion primitive planner in voxel map
 */
class MPMapUtil : public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPMapUtil(bool verbose);
    ///Set map util
    void setMapUtil(std::shared_ptr<MPL::VoxelMapUtil>& map_util);
    ///Get linked voxels
    vec_Vec3f getLinkedNodes() const;
    ///Update tree according to the new blocked nodes
    vec_Vec3f updateBlockedNodes(const vec_Vec3i& pns);
    ///Update tree according to the new cleared nodes
    vec_Vec3f updateClearedNodes(const vec_Vec3i& pns);

  protected:
    std::shared_ptr<MPL::VoxelMapUtil> map_util_;
    /// Linked map that records voxel and primitives passed through it
    mutable linkedHashMap lhm_;
};


