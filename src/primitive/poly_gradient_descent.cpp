#include <motion_primitive_library/primitive/poly_gradient_descent.h>
template <int Dim>
PolyGradientDescent<Dim>::PolyGradientDescent(unsigned int smooth_derivative_order,
                            unsigned int minimize_derivative, bool debug)
    : PolySolver<Dim>(smooth_derivative_order, minimize_derivative, debug) {
}

template <int Dim>
bool PolyGradientDescent<Dim>::gradient_descent(vec_E<Waypoint<Dim>> waypoints,
    const std::vector<double>& dts, int n) {
  this->solve(waypoints, dts);
  if(n == 0)
    return true;

  const unsigned int N = this->N_;
  const unsigned int num_waypoints = waypoints.size();
  const unsigned int num_segments = num_waypoints - 1;
  const unsigned int smooth_derivative_order = N / 2 - 1;
  const unsigned int num_total_derivatives = num_waypoints * N / 2;
  unsigned int num_fixed_derivatives = 0;
  // only consider the start and end derivatives to be fixed
  for(unsigned int i = 0; i < waypoints.size(); i++) {
    if(i > 0 && i < waypoints.size() - 1) {
      waypoints[i].use_pos = false;
      waypoints[i].use_vel = false;
      waypoints[i].use_acc = false;
      waypoints[i].use_jrk = false;
      continue;
    }
    if(waypoints[i].use_pos && smooth_derivative_order >= 0) 
      num_fixed_derivatives++;
    if(waypoints[i].use_vel && smooth_derivative_order >= 1)
      num_fixed_derivatives++;
    if(waypoints[i].use_acc && smooth_derivative_order >= 2)
      num_fixed_derivatives++;
    if(waypoints[i].use_jrk && smooth_derivative_order >= 3)
      num_fixed_derivatives++;
  }

  const unsigned int num_free_derivatives = num_total_derivatives - num_fixed_derivatives;
  // create permutation_table
  std::vector<std::pair<unsigned int, unsigned int>> permutation_table;
  unsigned int raw_cnt = 0;
  unsigned int fix_cnt = 0;
  unsigned int free_cnt = 0;
  unsigned int id = 0;
  for(const auto& it: waypoints) {
    if(it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if(it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    }
    else if(!it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, num_fixed_derivatives+free_cnt));
      if(id > 0 && id < num_waypoints-1)
        permutation_table.push_back(std::make_pair(raw_cnt+N/2, num_fixed_derivatives+free_cnt));
      raw_cnt++;
      free_cnt++;
    }

    if(id > 0 && id < num_waypoints-1)
      raw_cnt += N/2;
    id ++;
  }

  // M
  MatDf M = MatDf::Zero(num_segments * N, num_waypoints * N / 2);
  for(const auto& it: permutation_table)
    M(it.first, it.second) = 1;


  // End derivatives
  MatDNf<Dim> D = MatDNf<Dim>(num_waypoints * N / 2, Dim);
  for(const auto& it: permutation_table) {
    int id = std::floor((it.first + N / 2) / N);
    int derivative = it.first % (N / 2);
    if(derivative == 0)
      D.row(it.second) = waypoints[id].pos.transpose();
    else if(derivative == 1)
      D.row(it.second) = waypoints[id].vel.transpose();
    else if(derivative == 2)
      D.row(it.second) = waypoints[id].acc.transpose();
    else if(derivative == 3)
      D.row(it.second) = waypoints[id].jrk.transpose();
  }

  const MatDf A = this->A_;
  const MatDf Q = this->Q_;

  // L
  MatDf L = A.partialPivLu().solve(M);
  // R
  MatDf R = L.transpose() * Q * L;
  // V
  MatDf V = MatDf::Zero(num_segments * N, num_segments * N);
  for(unsigned int j = 0; j < num_segments; j++) {
    for(unsigned int i = 0; i < N-1; i++) 
      V(j * N + i, j * N + i + 1) = i+1;
  }

  int m = m_sample_;
  // T
  MatDf T = MatDf::Zero(num_segments * (m+1), num_segments * N);
  // dT
  MatDf deltaT(num_segments*(m+1), 1);
  for(unsigned int i = 0; i < num_segments; i++) {
    double dt = dts[i]/m;
    for(int j = 0; j <= m; j++) {
      T.block(i*(m+1)+j, i*N, 1, N) = (getT(N, j*dt)).transpose();
      deltaT(i*(m+1)+j) = dt;
    }
  }

  if(0) {
    std::cout << "T:\n" << T << std::endl;
    std::cout << "V:\n" << V << std::endl;

  }

  for(int i = 0; i < n; i++) {
    double J = gradient_descent(num_fixed_derivatives, num_free_derivatives,
        L, T, deltaT, V, R, D);
    std::cout << "i: " << i << " J: " << J << std::endl;
  }

  this->ptraj_->clear();
  this->ptraj_->addTime(dts);
  MatDNf<Dim> d = M * D;
  for (unsigned int i = 0; i < num_segments; i++) {
    const MatDNf<Dim> p = A.block(i * N, i * N, N, N)
      .partialPivLu()
      .solve(d.block(i * N, 0, N, Dim));
    this->ptraj_->addCoeff(p);
  }
 
  B_.clear();
  return true;
}

template <int Dim>
double PolyGradientDescent<Dim>::gradient_descent(int num_fixed_derivatives, int num_free_derivatives,
    const MatDf& L, const MatDf& T, const MatDf& deltaT, const MatDf& V, const MatDf& R,
    MatDNf<Dim>& D) {
  // current coefficients P
  const MatDNf<Dim> P = L*D;
  // Lpp
  MatDf Lpp = L.rightCols(num_free_derivatives);

  std::pair<MatDf, MatDf> Cs = get_Jc(T, deltaT, V, P, Lpp);
  // collision cost Jc
  MatDf Jc = Cs.first;
  // collision cost derivative Jc_d
  MatDf Jc_d = Cs.second;

  // fixed End Derivatives
  MatDNf<Dim> Df = D.topRows(num_fixed_derivatives);
  // free End Derivatives
  MatDNf<Dim> Dp = D.bottomRows(num_free_derivatives);

  // Rpp
  const MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
      num_free_derivatives, num_free_derivatives);
  // Rfp
  const MatDf Rfp = R.block(0, num_fixed_derivatives, num_fixed_derivatives,
      num_free_derivatives);

  if(B_.empty()) {
    for(int i = 0; i < Dim; i++)
      B_.push_back(MatDf::Identity(Dp.rows(), Dp.rows()));
  }

  double total_cost = 0;
  double beta = 0.0001;
  for(int dim = 0; dim < Dim; dim++) {
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
      if(alpha <= 1e-10)
        break;
      D.block(num_fixed_derivatives, dim, num_free_derivatives, 1) = 
        Dp.col(dim) + alpha * pk;
      Cs = get_Jc(T, deltaT, V, L*D, Lpp);
      J_new = D.col(dim).transpose() * R * D.col(dim) + Cs.first;
    }
    total_cost += J_new(0,0);
    const MatDf sk = alpha * pk;
    MatDf xk_new = Dp.col(dim) + sk;
    D.block(num_fixed_derivatives, dim, num_free_derivatives, 1) = xk_new;
    Cs = get_Jc(T, deltaT, V, L*D, Lpp);
    MatDf yk = (2*Df.col(dim).transpose()*Rfp+2*xk_new.transpose()*Rpp+Cs.second.row(dim)-gk).transpose();
    if(sk.norm() != 0) {
      B_[dim] += yk*yk.transpose()/(yk.transpose()*sk)(0, 0) - 
        B_[dim]*sk*sk.transpose()*B_[dim] / (sk.transpose() * B_[dim]*sk)(0, 0);
    }
  }

  return total_cost;
}

template <int Dim>
double PolyGradientDescent<Dim>::get_c(const Vecf<Dim>& pt, const Vecf<Dim>& ref_pt) {
  double d = (pt - ref_pt).norm();
  if(d > epsilon_)
    return 0;
  else
    return 0.5*epsilon_*(d-epsilon_)*(d-epsilon_);
}

template <int Dim>
Vecf<Dim> PolyGradientDescent<Dim>::get_c_derivative(const Vecf<Dim>& pt, const vec_Vecf<Dim>& ref_pts) {
  Vecf<Dim> der = Vecf<Dim>::Zero();
  int cnt = 0;
  for(const auto& ref_pt: ref_pts) {
    double d = (pt - ref_pt).norm();
    if(d < epsilon_) {
      der += (d-epsilon_)/(d*epsilon_)*(pt-ref_pt);
      cnt ++;
    }
  }
  if(cnt > 0)
    der /= cnt;
  return der;
}
 

template <int Dim>
VecDf PolyGradientDescent<Dim>::getT(int N, double t) {
  VecDf vec(N);
  for(int i = 0; i < N; i++)
    vec(i) = power(t, i);
  return vec;
}

template <int Dim>
std::pair<MatDf, MatDf> PolyGradientDescent<Dim>::get_Jc(const MatDf& T, const VecDf& deltaT, 
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
    double dist = std::numeric_limits<double>::infinity();
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

  MatDf Jc = wc_ * Cv.transpose() * deltaT;
  MatDf Jc_d = wc_ * deltaC.transpose()*T*Lpp +
    C.transpose() * T*V*Lpp;

  //std::cout << "P: \n" << P << std::endl;
 // std::cout << "deltaC: \n" << deltaC << std::endl;
 // std::cout << "C: \n" << C << std::endl;

  return std::make_pair(Jc, Jc_d);
}

template class PolyGradientDescent<2>;

template class PolyGradientDescent<3>;
