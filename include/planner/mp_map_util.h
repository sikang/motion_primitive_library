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
    /**
     * @brief Update edge costs according to the new blocked nodes
     * @param pns the new occupied voxels 
     *
     * The function returns affected primitives for debug purpose
     */
    std::vector<Primitive> updateBlockedNodes(const vec_Vec3i& pns);
    /**
     * @brief Update edge costs according to the new cleared nodes
     * @param pns the new cleared voxels 
     *
     * The function returns affected primitives for debug purpose
     */
    std::vector<Primitive> updateClearedNodes(const vec_Vec3i& pns);

  protected:
    ///Map util
    std::shared_ptr<MPL::VoxelMapUtil> map_util_;
    ///Linked table that records voxel and corresponding primitives passed through it
    mutable linkedHashMap lhm_;
};


