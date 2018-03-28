#include <motion_primitive_library/primitive/poly_solver.h>
template <int Dim>
PolySolver<Dim>::PolySolver(unsigned int smooth_derivative_order,
                            unsigned int minimize_derivative, bool debug)
    : N_(2 * (smooth_derivative_order + 1)), R_(minimize_derivative),
      debug_(debug) {
  ptraj_ = std::make_shared<PolyTraj<Dim>>();
}

template <int Dim>
std::shared_ptr<PolyTraj<Dim>> PolySolver<Dim>::getTrajectory() { return ptraj_; }

template <int Dim>
bool PolySolver<Dim>::solve(const vec_E<Waypoint<Dim>>& waypoints,
                            const std::vector<decimal_t>& dts) {
  B_.clear();
  ptraj_->clear();
  ptraj_->addTime(dts);

  const unsigned int num_waypoints = waypoints.size();
  const unsigned int num_segments = num_waypoints - 1;
  if(num_waypoints < 2)
    return false;
  if(debug_) {
    for(unsigned int i = 0; i < num_waypoints; i++)
      waypoints[i].print("waypoint"+std::to_string(i)+":");

  }

  MatDf A = MatDf::Zero(num_segments * N_, num_segments * N_);
  MatDf Q = MatDf::Zero(num_segments * N_, num_segments * N_);
  for (unsigned int i = 0; i < num_segments; i++) {
    decimal_t seg_time = dts[i];
    // n column
    for (unsigned int n = 0; n < N_; n++) {
      // A_0
      if (n < N_ / 2) {
        int val = 1;
        for (unsigned int m = 0; m < n; m++)
          val *= (n - m);
        A(i * N_ + n, i * N_ + n) = val;
      }
      // A_T
      for (unsigned int r = 0; r < N_ / 2; r++) {
        if (r <= n) {
          int val = 1;
          for (unsigned int m = 0; m < r; m++)
            val *= (n - m);
          A(i * N_ + N_ / 2 + r, i * N_ + n) = val * power(seg_time, n - r);
        }
      }
      // Q
      for (unsigned int r = 0; r < N_; r++) {
        if (r >= R_ && n >= R_) {
          int val = 1;
          for (unsigned int m = 0; m < R_; m++)
            val *= (r - m) * (n - m);
          Q(i * N_ + r, i * N_ + n) = val *
                                      power(seg_time, r + n - 2 * R_ + 1) /
                                      (r + n - 2 * R_ + 1);
        }
      }
    }
  }

  const unsigned int smooth_derivative_order = N_ / 2 - 1;
  const unsigned int num_total_derivatives = num_waypoints * N_ / 2;
  unsigned int num_fixed_derivatives = 0;
  for(const auto& it: waypoints) {
    if(it.use_pos && smooth_derivative_order >= 0) 
      num_fixed_derivatives++;
    if(it.use_vel && smooth_derivative_order >= 1)
      num_fixed_derivatives++;
    if(it.use_acc && smooth_derivative_order >= 2)
      num_fixed_derivatives++;
    if(it.use_jrk && smooth_derivative_order >= 3)
      num_fixed_derivatives++;
  }
  const unsigned int num_free_derivatives = num_total_derivatives - num_fixed_derivatives;
  if (debug_)
    printf("num_fixed_derivatives: %d, num_free_derivatives: %d\n",
           num_fixed_derivatives, num_free_derivatives);

  std::vector<std::pair<unsigned int, unsigned int>> permutation_table;
  unsigned int raw_cnt = 0;
  unsigned int fix_cnt = 0;
  unsigned int free_cnt = 0;
  unsigned int id = 0;
  for(const auto& it: waypoints) {
    if(it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }

    if(id > 0 && id < num_waypoints-1)
      raw_cnt += N_/2;
    id ++;
  }

  if(debug_) {
    std::cout << "permutation_table: \n" << std::endl;
    for(auto it: permutation_table) 
      std::cout << "old id: " << it.first << " new_id: " << it.second << std::endl;
  }

  // M
  MatDf M = MatDf::Zero(num_segments * N_, num_waypoints * N_ / 2);
  for(const auto& it: permutation_table)
    M(it.first, it.second) = 1;

  // Eigen::MatrixXf A_inv = A.inverse();
  MatDf A_inv_M = A.partialPivLu().solve(M);
  MatDf R = A_inv_M.transpose() * Q * A_inv_M;
  if (debug_) {
    std::cout << "A:\n" << A << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "M:\n" << M << std::endl;
    std::cout << "A_inv_M:\n" << A_inv_M << std::endl;
    std::cout << "R:\n" << R << std::endl;
  }
  MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
      num_free_derivatives, num_free_derivatives);
  MatDf Rpf = R.block(num_fixed_derivatives, 0, num_free_derivatives,
      num_fixed_derivatives);

  // Fixed derivatives
  MatDNf<Dim> Df = MatDNf<Dim>(num_fixed_derivatives, Dim);
  for(const auto& it: permutation_table) {
    if(it.second < num_fixed_derivatives) {
      int id = std::floor((it.first + N_/2) / N_);
      int derivative = it.first % (N_ / 2);
      if(derivative == 0)
        Df.row(it.second) = waypoints[id].pos.transpose();
      else if(derivative == 1)
        Df.row(it.second) = waypoints[id].vel.transpose();
      else if(derivative == 2)
        Df.row(it.second) = waypoints[id].acc.transpose();
      else if(derivative == 3)
        Df.row(it.second) = waypoints[id].jrk.transpose();
    }
  }

  if (debug_)
    std::cout << "Df:\n" << Df << std::endl;
  MatDNf<Dim> D = MatDNf<Dim>(num_waypoints * N_ / 2, Dim);
  D.topRows(num_fixed_derivatives) = Df;

  if (num_waypoints > 2 && num_free_derivatives > 0) {
    MatDNf<Dim> Dp = -Rpp.partialPivLu().solve(Rpf * Df);
    // std::cout << "Dp:\n" << Dp << std::endl;
    D.bottomRows(num_free_derivatives) = Dp;
  }

  MatDNf<Dim> d = M * D;
  if (debug_)
    std::cout << "d:\n" << d << std::endl;
  for (unsigned int i = 0; i < num_segments; i++) {
    const MatDNf<Dim> p = A.block(i * N_, i * N_, N_, N_)
      .partialPivLu()
      .solve(d.block(i * N_, 0, N_, Dim));
    if (debug_)
      std::cout << "p:\n" << p << std::endl;
    ptraj_->addCoeff(p);
  }
  return true;
}

template <int Dim>
bool PolySolver<Dim>::gradient_descent(const vec_E<Waypoint<Dim>>& waypoints,
    const std::vector<double>& dts, int m_sample) {
  const MatDNf<Dim> P = ptraj_->p();
  ptraj_->clear();
  ptraj_->addTime(dts);


  const unsigned int num_waypoints = waypoints.size();
  const unsigned int num_segments = num_waypoints - 1;
  if(num_waypoints < 2)
    return false;
  if(debug_) {
    for(unsigned int i = 0; i < num_waypoints; i++)
      waypoints[i].print("waypoint"+std::to_string(i)+":");
  }

  MatDf A = MatDf::Zero(num_segments * N_, num_segments * N_);
  MatDf Q = MatDf::Zero(num_segments * N_, num_segments * N_);
  for (unsigned int i = 0; i < num_segments; i++) {
    decimal_t seg_time = dts[i];
    // n column
    for (unsigned int n = 0; n < N_; n++) {
      // A_0
      if (n < N_ / 2) {
        int val = 1;
        for (unsigned int m = 0; m < n; m++)
          val *= (n - m);
        A(i * N_ + n, i * N_ + n) = val;
      }
      // A_T
      for (unsigned int r = 0; r < N_ / 2; r++) {
        if (r <= n) {
          int val = 1;
          for (unsigned int m = 0; m < r; m++)
            val *= (n - m);
          A(i * N_ + N_ / 2 + r, i * N_ + n) = val * power(seg_time, n - r);
        }
      }
      // Q
      for (unsigned int r = 0; r < N_; r++) {
        if (r >= R_ && n >= R_) {
          int val = 1;
          for (unsigned int m = 0; m < R_; m++)
            val *= (r - m) * (n - m);
          Q(i * N_ + r, i * N_ + n) = val *
                                      power(seg_time, r + n - 2 * R_ + 1) /
                                      (r + n - 2 * R_ + 1);
        }
      }
    }
  }

  const unsigned int smooth_derivative_order = N_ / 2 - 1;
  const unsigned int num_total_derivatives = num_waypoints * N_ / 2;
  unsigned int num_fixed_derivatives = 0;
  for(const auto& it: waypoints) {
    if(it.use_pos && smooth_derivative_order >= 0) 
      num_fixed_derivatives++;
    if(it.use_vel && smooth_derivative_order >= 1)
      num_fixed_derivatives++;
    if(it.use_acc && smooth_derivative_order >= 2)
      num_fixed_derivatives++;
    if(it.use_jrk && smooth_derivative_order >= 3)
      num_fixed_derivatives++;
  }
  const unsigned int num_free_derivatives = num_total_derivatives - num_fixed_derivatives;
  std::vector<std::pair<unsigned int, unsigned int>> permutation_table;
  unsigned int raw_cnt = 0;
  unsigned int fix_cnt = 0;
  unsigned int free_cnt = 0;
  unsigned int id = 0;
  for(const auto& it: waypoints) {
    if(it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N_/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }

    if(id > 0 && id < num_waypoints-1)
      raw_cnt += N_/2;
    id ++;
  }

  // M
  MatDf M = MatDf::Zero(num_segments * N_, num_waypoints * N_ / 2);
  for(const auto& it: permutation_table)
    M(it.first, it.second) = 1;

  MatDf L = A.partialPivLu().solve(M);
  MatDf R = L.transpose() * Q * L;
  // V
  MatDf V = MatDf::Zero(num_segments * N_, num_segments * N_);
  for(unsigned int j = 0; j < num_segments; j++) {
    for(unsigned int i = 0; i < N_-1; i++) 
      V(j*N_+i, j*N_+i+1) = i+1;
  }

  // T
  int m = m_sample;
  MatDf T = MatDf::Zero(num_segments*(m+1), num_segments*N_);
  MatDf deltaT(num_segments*(m+1), 1);
  for(unsigned int i = 0; i < num_segments; i++) {
    double dt = dts[i]/m;
    for(int j = 0; j <= m; j++) {
      T.block(i*(m+1)+j, i*N_, 1, N_) = (getT(N_, j*dt)).transpose();
      deltaT(i*(m+1)+j) = dt;
    }
  }
  // Lpp
  MatDf Lpp = L.rightCols(num_free_derivatives);

  std::pair<MatDf, MatDf> Cs = get_Jc(T, deltaT, V, P, Lpp);
  MatDf Jc = Cs.first;
  MatDf Jc_d = Cs.second;

  if(debug_) {
    std::cout << "T:\n" << T << std::endl;
    std::cout << "V:\n" << V << std::endl;
    std::cout << "Jc:\n" << Jc << std::endl;
    std::cout << "Jc_d:\n" << Jc_d << std::endl;
  }


  // End derivatives
  MatDNf<Dim> D = MatDNf<Dim>(num_waypoints * N_ / 2, Dim);
  for(const auto& it: permutation_table) {
    int id = std::floor((it.first + N_/2) / N_);
    int derivative = it.first % (N_ / 2);
    if(derivative == 0)
      D.row(it.second) = waypoints[id].pos.transpose();
    else if(derivative == 1)
      D.row(it.second) = waypoints[id].vel.transpose();
    else if(derivative == 2)
      D.row(it.second) = waypoints[id].acc.transpose();
    else if(derivative == 3)
      D.row(it.second) = waypoints[id].jrk.transpose();
  }
  MatDNf<Dim> Df = D.topRows(num_fixed_derivatives);
  MatDNf<Dim> Dp = D.bottomRows(num_free_derivatives);

  const MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
      num_free_derivatives, num_free_derivatives);
  const MatDf Rfp = R.block(0, num_fixed_derivatives, num_fixed_derivatives,
      num_free_derivatives);

  if(B_.empty()) {
    for(int i = 0; i < Dim; i++)
      B_.push_back(MatDf::Identity(Dp.rows(), Dp.rows()));
  }
 
  for(int dim = 0; dim < Dim; dim++) {
    double beta = 0.0001;
    double alpha = 1;
    MatDf gk = 2*Df.col(dim).transpose()*Rfp+2*Dp.col(dim).transpose()*Rpp + Jc_d.row(dim);
    MatDf pk = -B_[dim].inverse() * gk.transpose();
    const MatDf J = D.col(dim).transpose() * R * D.col(dim) + Jc;
    D.block(num_fixed_derivatives, dim, num_free_derivatives, 1) = 
      Dp.col(dim) + alpha * pk;
    Cs = get_Jc(T, deltaT, V, L*D, Lpp);
    MatDf J_new = D.col(dim).transpose() * R * D.col(dim) + Cs.first;
    // line search
    while(J_new(0, 0) > (J + alpha*beta*gk*pk)(0, 0)) {
      alpha *= 0.5;
      //std::cout << "alpha: " << alpha << std::endl;
      if(alpha <= 1e-10)
        break;
      D.block(num_fixed_derivatives, dim, num_free_derivatives, 1) = 
        Dp.col(dim) + alpha * pk;
      Cs = get_Jc(T, deltaT, V, L*D, Lpp);
      J_new = D.col(dim).transpose() * R * D.col(dim) + Cs.first;
    }
    MatDf sk = alpha * pk;
    MatDf xk_new = Dp.col(dim) + sk;
    D.block(num_fixed_derivatives, dim, num_free_derivatives, 1) = xk_new;
    Cs = get_Jc(T, deltaT, V, L*D, Lpp);
    MatDf yk = (2*Df.col(dim).transpose()*Rfp+2*xk_new.transpose()*Rpp+Cs.second.row(dim)-gk).transpose();
    B_[dim] += yk*yk.transpose()/(yk.transpose()*sk)(0, 0) - 
      B_[dim]*sk*sk.transpose()*B_[dim] / (sk.transpose() * B_[dim]*sk)(0, 0);
  }



  MatDNf<Dim> d = M * D;
  for (unsigned int i = 0; i < num_segments; i++) {
    const MatDNf<Dim> p = A.block(i * N_, i * N_, N_, N_)
      .partialPivLu()
      .solve(d.block(i * N_, 0, N_, Dim));
    ptraj_->addCoeff(p);
  }
 

  if (debug_) {
    std::cout << "Df:\n" << Df << std::endl;
    std::cout << "Dp:\n" << Dp << std::endl;
    std::cout << "Dp_new:\n" << D.bottomRows(num_free_derivatives) << std::endl;
    std::cout << "Dp*:\n" << -Rpp.partialPivLu().solve(Rfp.transpose() * Df) << std::endl;
  }


  return false;
}

template <int Dim>
double PolySolver<Dim>::get_c(const Vecf<Dim>& pt, const Vecf<Dim>& ref_pt) {
  double epsilon = 0.5;
  double d = (pt - ref_pt).norm();
  if(d > epsilon)
    return 0;
  else
    return 0.5*epsilon*(d-epsilon)*(d-epsilon);
}

template <int Dim>
Vecf<Dim> PolySolver<Dim>::get_c_derivative(const Vecf<Dim>& pt, const vec_Vecf<Dim>& ref_pts) {
  double epsilon = 0.5;
  Vecf<Dim> der = Vecf<Dim>::Zero();
  int cnt = 0;
  for(const auto& ref_pt: ref_pts) {
    double d = (pt - ref_pt).norm();
    if(d < epsilon) {
      der += (d-epsilon)/(d*epsilon)*(pt-ref_pt);
      cnt ++;
    }
  }
  if(cnt > 0)
    der /= cnt;
  return der;
}
 

template <int Dim>
VecDf PolySolver<Dim>::getT(int N, double t) {
  VecDf vec(N);
  for(int i = 0; i < N; i++)
    vec(i) = power(t, i);
  return vec;
}

template <int Dim>
std::pair<MatDf, MatDf> PolySolver<Dim>::get_Jc(const MatDf& T, const VecDf& deltaT, 
    const MatDf& V, const MatDf& P, const MatDf& Lpp) {
  // f
  MatDf f = T*P;
  // v
  MatDf v = T*V*P;

  MatDf deltaC(v.rows(), Dim);
  MatDf C(v.rows(), Dim);
  VecDf Cv(v.rows());
  for(int i = 0; i < v.rows(); i++) {
    double v_abs = v.row(i).norm();
    const Vecf<Dim> pt = f.row(i);
    Vecf<Dim> closest_pt;
    double dist = 10000;
    for(const auto& it: obs_) {
      if((it-pt).norm() < dist) {
        closest_pt = it;
        dist = (it-pt).norm();
      }
    }

    double c = get_c(pt, closest_pt);
    if(v_abs == 0)
      C.row(i) = Vecf<Dim>::Zero(); 
    else 
      C.row(i) = c*v.row(i)/v_abs*deltaT(i);
    deltaC.row(i) = get_c_derivative(pt, obs_).transpose()*v_abs*deltaT(i);
    Cv(i) = v_abs*c;
  }

  double wc = 10000;
  MatDf Jc = wc * Cv.transpose() * deltaT;
  MatDf Jc_d = wc * deltaC.transpose()*T*Lpp +
    C.transpose() * T*V*Lpp;

  //std::cout << "deltaC: \n" << deltaC << std::endl;
  //std::cout << "C: \n" << C << std::endl;

  return std::make_pair(Jc, Jc_d);
}

template class PolySolver<2>;

template class PolySolver<3>;
