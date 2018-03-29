/**
 * @file poly_gradient_descent.h
 * @brief Trajectory generator back-end
 */
#ifndef POLY_GRADIENT_DESCENT_H
#define POLY_GRADIENT_DESCENT_H

#include <motion_primitive_library/primitive/poly_solver.h>

/**
 * @brief Trajectory generator back-end class
 *
 * Given intermediate waypoints and associated time allocation, generate the n-th order polynomials
 */
template <int Dim>
class PolyGradientDescent : public PolySolver<Dim> {
  public:
    /**
     * @brief Simple constructor
     * @param smooth_derivative_order The max derivative we want continuous
     * @param minimize_derivative The derivative to minimize
     */
    PolyGradientDescent(unsigned int smooth_derivative_order, unsigned int minimize_derivative, bool debug = false);
    void set_obs(const vec_Vecf<Dim>& obs) { obs_ = obs; }
    void set_m_sample(int m) { m_sample_ = m; }
    void set_epsilon(double e) { epsilon_ = e; }
    void set_wc(double wc) { wc_ = wc; }

    bool gradient_descent(vec_E<Waypoint<Dim>> waypoints,
        const std::vector<double>& dts, int n = 10);
 
 private:
    double gradient_descent(int num_fixed_derivatives, int num_free_derivatives,
        const MatDf& L, const MatDf& T, const MatDf& deltaT, const MatDf& V, const MatDf& R,
        MatDNf<Dim>& D);
    std::pair<MatDf, MatDf> get_Jc(const MatDf& T, const VecDf& deltaT, const MatDf& V, const MatDf& P, const MatDf& Lpp);
    double get_c(const Vecf<Dim>& pt, const Vecf<Dim>& ref_pt);
    Vecf<Dim> get_c_derivative(const Vecf<Dim>& pt, const vec_Vecf<Dim>& ref_pts);
    VecDf getT(int N, double t);
    vec_E<MatDf> B_;
    vec_Vecf<Dim> obs_;
    int m_sample_ = 5;
    double epsilon_ = 1.0;
    double wc_ = 1000;
};

#endif
