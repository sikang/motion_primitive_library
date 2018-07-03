#include <motion_primitive_library/planner/mp_distance_map_util.h>

using namespace MPL;

template <int Dim> MPDistanceMapUtil<Dim>::MPDistanceMapUtil(bool verbose) :
MPMapUtil<Dim>(verbose) {
  this->planner_verbose_ = verbose;
  if (this->planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPDistanceMapUtil] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim>
void MPDistanceMapUtil<Dim>::setMapUtil(std::shared_ptr<MapUtil<Dim>> &map_util) {
  this->ENV_.reset(new MPL::env_distance_map<Dim>(map_util));
  this->map_util_ = map_util;
}

template <int Dim>
void MPDistanceMapUtil<Dim>::setDistanceRadius(const Vecf<Dim>& radius) {
  distance_radius_ = radius;
}

template <int Dim>
void MPDistanceMapUtil<Dim>::setMapRange(const Vecf<Dim>& range) {
  map_range_ = range;
}

template <int Dim>
void MPDistanceMapUtil<Dim>::setPotentialWeight(decimal_t w) {
  this->ENV_->set_potential_weight(w);
}

template <int Dim>
void MPDistanceMapUtil<Dim>::setGradientWeight(decimal_t w) {
  this->ENV_->set_gradient_weight(w);
}

template <int Dim>
vec_Vec3f MPDistanceMapUtil<Dim>::getPotentialCloud(decimal_t h_max) {
  const auto data = this->map_util_->getMap();
  const auto dim = this->map_util_->getDim();
  const decimal_t ratio = h_max / H_MAX;
  vec_Vec3f ps;

  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = 0; n(0) < dim(0); n(0)++) {
      for(n(1) = 0; n(1) < dim(1); n(1)++) {
        int idx = this->map_util_->getIndex(n);
        if(data[idx] > 0) {
          decimal_t h = (decimal_t) data[idx] * ratio;
          Vecf<Dim> pt2d = this->map_util_->intToFloat(n);
          ps.push_back(Vec3f(pt2d(0), pt2d(1), h));
        }
      }
    }
  }
  else {
    for(n(0) = 0; n(0) < dim(0); n(0)++) {
      for(n(1) = 0; n(1) < dim(1); n(1)++) {
        for(n(2) = 0; n(2) < dim(2); n(2)++) {
          int idx = this->map_util_->getIndex(n);
          if(data[idx] > 0)  {
            auto pt = this->map_util_->intToFloat(n);
            Vec3f p;
            p << pt(0), pt(1), pt(2);
            ps.push_back(p);
          }
        }
      }
    }
  }
  return ps;
}


template <int Dim>
vec_Vec3f MPDistanceMapUtil<Dim>::getGradientCloud(decimal_t h_max, int i) {
  const auto data = this->map_util_->getMap();
  const auto dim = this->map_util_->getDim();
  const decimal_t ratio = h_max / H_MAX;
  vec_Vec3f ps;
  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = 0; n(0) < dim(0); n(0)++) {
      for(n(1) = 0; n(1) < dim(1); n(1)++) {
        int idx = this->map_util_->getIndex(n);
        if(gradient_map_[idx].norm() > 0) {
          Vecf<Dim> pt2d = this->map_util_->intToFloat(n);
          ps.push_back(Vec3f(pt2d(0), pt2d(1), gradient_map_[idx](i)*ratio));
        }
      }
    }

  }
  return ps;
}

template <int Dim>
vec_E<Vecf<Dim>> MPDistanceMapUtil<Dim>::calculateGradient(const Veci<Dim>& coord1, const Veci<Dim>& coord2) {
  const auto dim = this->map_util_->getDim();
  const auto cmap = this->map_util_->getMap();

  int rn = std::ceil(distance_radius_(0) / this->map_util_->getRes());

  vec_E<Vecf<Dim>> g(dim(0) * dim(1), Vecf<Dim>::Zero());
  Veci<Dim> n;
  for(n(0) = coord1(0)-rn; n(0) < coord2(0)+rn; n(0)++) {
    for(n(1) = coord1(1)-rn; n(1) < coord2(1)+rn; n(1)++) {
      if(this->map_util_->isOutside(n))
        continue;
      int idx = this->map_util_->getIndex(n);
      auto nx1 = n; nx1(0) -= 1;
      auto nx2 = n; nx2(0) += 1;
      int idx_x1 = this->map_util_->isOutside(nx1) ? -1 : this->map_util_->getIndex(nx1);
      int idx_x2 = this->map_util_->isOutside(nx2) ? -1 : this->map_util_->getIndex(nx2);

      if(idx_x1 >= 0 && idx_x2 >= 0)
        g[idx](0) = 0.5 * (cmap[idx_x2] - cmap[idx_x1]);

      auto ny1 = n; ny1(1) -= 1;
      auto ny2 = n; ny2(1) += 1;
      int idx_y1 = this->map_util_->isOutside(ny1) ? -1 : this->map_util_->getIndex(ny1);
      int idx_y2 = this->map_util_->isOutside(ny2) ? -1 : this->map_util_->getIndex(ny2);

      if(idx_y1 >= 0 && idx_y2 >= 0)
        g[idx](1) = 0.5 * (cmap[idx_y2] - cmap[idx_y1]);

    }
  }

  return g;
}

template <int Dim>
void MPDistanceMapUtil<Dim>::createMask(int pow) {
  distance_mask_.clear();
  // create mask
  decimal_t res = this->map_util_->getRes();
  decimal_t h_max = H_MAX;
  int rn = std::ceil(distance_radius_(0) / res);
  // printf("rn: %d\n", rn);
  // printf("hn: %d\n", hn);
  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = -rn; n(0) <= rn; n(0)++) {
      for (n(1) = -rn; n(1) <= rn; n(1)++) {
        if (std::hypot(n(0), n(1)) > rn)
          continue;
        decimal_t h =
          h_max * std::pow((1 - (decimal_t)std::hypot(n(0), n(1)) / rn), pow);
        if (h > 1e-3)
          distance_mask_.push_back(std::make_pair(n, (int8_t)h));
      }
    }
  } else {
    int hn = std::ceil(distance_radius_(2) / res);
    for (n(0) = -rn; n(0) <= rn; n(0)++) {
      for (n(1) = -rn; n(1) <= rn; n(1)++) {
        for (n(2) = -hn; n(2) <= hn; n(2)++) {
          if (std::hypot(n(0), n(1)) > rn)
            continue;
          decimal_t h =
            h_max * std::pow((1 - (decimal_t)std::hypot(n(0), n(1)) / rn) *
                             (1 - (decimal_t)std::abs(n(2)) / hn),
                             pow);
          if (h > 1e-3)
            distance_mask_.push_back(std::make_pair(n, (int8_t)h));
        }
      }
    }
  }
}

template <int Dim>
void MPDistanceMapUtil<Dim>::updateDistanceMap(const Vecf<Dim>& pos) {
  createMask(1);
  // compute a 2D local distance map
  const auto dim = this->map_util_->getDim();
  Veci<Dim> coord1 = Veci<Dim>::Zero();
  Veci<Dim> coord2 = dim;
  if(map_range_.norm() > 0) {
    coord1 = this->map_util_->floatToInt(pos - map_range_);
    coord2 = this->map_util_->floatToInt(pos + map_range_);
    for(int i = 0; i < Dim; i++) {
      if(coord1(i) < 0)
        coord1(i) = 0;
      else if(coord1(i) >= dim(i))
        coord1(i) = dim(i) - 1;

      if(coord2(i) < 0)
        coord2(i) = 0;
      else if(coord2(i) >= dim(i))
        coord2(i) = dim(i) - 1;
    }
  }

  std::vector<int8_t> map = this->map_util_->getMap();
  auto distance_map = map;

  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
      for(n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
        int idx = this->map_util_->getIndex(n);
        if(map[idx] > 0) {
          distance_map[idx] = H_MAX;
          for(const auto& it: distance_mask_) {
            const Veci<Dim> new_n = n + std::get<0>(it);

            if(!this->map_util_->isOutside(new_n)) {
              const int new_idx = this->map_util_->getIndex(new_n);
              distance_map[new_idx] = std::max(distance_map[new_idx], std::get<1>(it));
            }
          }
        }
      }
    }
	}
	else {
		for(n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
			for(n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
				for(n(2) = coord1(2); n(2) < coord2(2); n(2)++) {
					int idx = this->map_util_->getIndex(n);
					if(map[idx] > 0) {
						distance_map[idx] = H_MAX;
						for(const auto& it: distance_mask_) {
							const Veci<Dim> new_n = n + std::get<0>(it);

							if(!this->map_util_->isOutside(new_n)) {
								const int new_idx = this->map_util_->getIndex(new_n);
                distance_map[new_idx] = std::max(distance_map[new_idx], std::get<1>(it));
							}
						}
					}
				}
			}
		}
	}

	this->map_util_->setMap(this->map_util_->getOrigin(), dim, distance_map, this->map_util_->getRes());
  this->ENV_->set_potential_map(this->map_util_->getMap());
  gradient_map_ = calculateGradient(coord1, coord2);
  this->ENV_->set_gradient_map(gradient_map_);
}


template class MPDistanceMapUtil<2>;

template class MPDistanceMapUtil<3>;
