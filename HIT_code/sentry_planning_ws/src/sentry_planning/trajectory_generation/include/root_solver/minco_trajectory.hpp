#ifndef SENTRY_PLANNING_MINCO_TRAJECTORY
#define SENTRY_PLANNING_MINCO_TRAJECTORY

/**
 * @file minco_trajectory.hpp
 * @brief MINCO (Minimum Control) trajectory representation — 5th-order
 *        (min-jerk) piecewise polynomial for 2D (x, y).
 *
 * Replaces CubicSpline for joint space-time optimization.
 * Each piece is a quintic polynomial with 6 coefficients:
 *   p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 *
 * The M-matrix system has dimension 3(N+1) x 3(N+1) with bandwidth 6,
 * mapping boundary conditions (position, velocity, acceleration at each
 * knot) to polynomial coefficients.
 *
 * Energy functional: E = Sum_i Int_0^{T_i} ||p''(t)||^2 dt  (jerk integral)
 *
 * Provides analytic gradients with respect to both waypoint positions
 * and segment durations — MINCO's key advantage over fixed-time cubic splines.
 */

#include <iostream>
#include <cfloat>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <cmath>
#include <vector>

class MincoTrajectory
{
public:
    MincoTrajectory() = default;
    ~MincoTrajectory() = default;

private:
    int N;                    // number of polynomial pieces
    Eigen::Vector2d headP;   // start position
    Eigen::Vector2d headV;   // start velocity
    Eigen::Vector2d headA;   // start acceleration
    Eigen::Vector2d tailP;   // end position
    Eigen::Vector2d tailV;   // end velocity
    Eigen::Vector2d tailA;   // end acceleration

    Eigen::VectorXd T;       // duration per piece (N)

    // Polynomial coefficients: 6 per piece per axis, stored as (6*N) x 2
    // Row layout: [c0, c1, c2, c3, c4, c5] for piece i at rows 6*i .. 6*i+5
    Eigen::MatrixX2d coeffs;

    // Intermediate M-matrix quantities for gradient computation
    // We store per-piece boundary values (pos, vel, acc at start of each piece)
    // for adjoint gradient propagation.
    Eigen::MatrixX2d boundaryPVA;  // (3*(N+1)) x 2: [p0,v0,a0, p1,v1,a1, ...]

public:
    /**
     * @brief Set boundary conditions and allocate storage.
     */
    inline void setConditions(const Eigen::Vector2d &headPos,
                              const Eigen::Vector2d &headVel,
                              const Eigen::Vector2d &headAcc,
                              const Eigen::Vector2d &tailPos,
                              const Eigen::Vector2d &tailVel,
                              const Eigen::Vector2d &tailAcc,
                              const int &pieceNum)
    {
        N = pieceNum;
        headP = headPos;
        headV = headVel;
        headA = headAcc;
        tailP = tailPos;
        tailV = tailVel;
        tailA = tailAcc;

        T.resize(N);
        coeffs.resize(6 * N, 2);
        boundaryPVA.resize(3 * (N + 1), 2);
    }

    /**
     * @brief Given inner waypoint positions and durations, build the MINCO
     *        trajectory by solving for polynomial coefficients.
     *
     * For min-jerk (order 5), each piece has 6 unknowns. We enforce:
     * - Continuity of position, velocity, acceleration at inner knots
     * - Boundary conditions at head and tail
     * - Minimum jerk (Int||p''||^2dt) energy
     *
     * We use the closed-form MINCO construction: given boundary PVA at each
     * knot, polynomial coefficients are determined directly per piece.
     * The inner waypoints are intermediate positions; inner velocities and
     * accelerations are chosen to minimize the jerk integral.
     *
     * This implementation uses the MINCO approach from Z. Wang et al.:
     * For each piece with duration T_i, boundary (p0,v0,a0) -> (p1,v1,a1),
     * the quintic polynomial coefficients are uniquely determined.
     * The optimal inner velocities and accelerations are found by solving
     * a banded linear system (the "M-matrix").
     */
    inline void setParameters(const Eigen::Ref<const Eigen::Matrix2Xd> &innerPs,
                              const Eigen::VectorXd &durations)
    {
        T = durations;
        const int innerN = N - 1;  // number of inner waypoints

        // Step 1: Set up boundary PVA
        // boundaryPVA layout: for knot k, rows 3*k, 3*k+1, 3*k+2 = pos, vel, acc
        boundaryPVA.setZero();
        boundaryPVA.row(0) = headP.transpose();
        boundaryPVA.row(1) = headV.transpose();
        boundaryPVA.row(2) = headA.transpose();
        boundaryPVA.row(3 * N)     = tailP.transpose();
        boundaryPVA.row(3 * N + 1) = tailV.transpose();
        boundaryPVA.row(3 * N + 2) = tailA.transpose();

        // Set position at inner knots
        for (int i = 0; i < innerN; i++) {
            boundaryPVA.row(3 * (i + 1)) = innerPs.col(i).transpose();
        }

        // Step 2: Solve for optimal inner velocities and accelerations
        // by minimizing jerk energy: Int||p''(t)||^2 dt.
        //
        // The optimal velocity and acceleration at interior knots satisfy
        // a block-tridiagonal system. We solve this via Thomas algorithm.
        //
        // For each piece i with duration T_i connecting (p_i, v_i, a_i) to
        // (p_{i+1}, v_{i+1}, a_{i+1}), the min-jerk cost gradient w.r.t.
        // the free variables (v_k, a_k) at interior knots yields a banded system.

        if (innerN > 0) {
            solveOptimalVelAcc(innerPs);
        }

        // Step 3: Compute polynomial coefficients for each piece
        computeCoeffs();
    }

    /**
     * @brief Compute jerk energy: E = Sum_i Int_0^{T_i} ||p''(t)||^2 dt
     * For quintic p(t) = c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5:
     *   p''(t) = 6*c3 + 24*c4*t + 60*c5*t^2
     *   ||p''||^2 = 36*c3^2 + 288*c3*c4*t + (576*c4^2 + 720*c3*c5)*t^2
     *           + 2880*c4*c5*t^3 + 3600*c5^2*t^4
     * Integrating from 0 to T:
     *   Int = 36*c3^2*T + 144*c3*c4*T^2 + (192*c4^2+240*c3*c5)*T^3
     *     + 720*c4*c5*T^4 + 720*c5^2*T^5
     */
    inline double getEnergy() const
    {
        double energy = 0.0;
        for (int i = 0; i < N; i++) {
            const double Ti = T(i);
            const double Ti2 = Ti * Ti;
            const double Ti3 = Ti2 * Ti;
            const double Ti4 = Ti3 * Ti;
            const double Ti5 = Ti4 * Ti;

            // coefficients for piece i
            const Eigen::Vector2d c3 = coeffs.row(6 * i + 3).transpose();
            const Eigen::Vector2d c4 = coeffs.row(6 * i + 4).transpose();
            const Eigen::Vector2d c5 = coeffs.row(6 * i + 5).transpose();

            energy += 36.0 * c3.squaredNorm() * Ti
                    + 144.0 * c3.dot(c4) * Ti2
                    + (192.0 * c4.squaredNorm() + 240.0 * c3.dot(c5)) * Ti3
                    + 720.0 * c4.dot(c5) * Ti4
                    + 720.0 * c5.squaredNorm() * Ti5;
        }
        return energy;
    }

    /**
     * @brief Gradient of jerk energy w.r.t. inner waypoint positions.
     * Uses adjoint method: dE/dq = (dc/dq)^T * (dE/dc).
     * Since changing inner waypoint positions affects the M-matrix solution
     * (the optimal velocities and accelerations), the gradient propagates
     * through the linear system via the adjoint.
     */
    inline void getGradWaypoints(Eigen::Matrix2Xd &gradByPoints) const
    {
        const int innerN = N - 1;
        gradByPoints.resize(2, innerN);
        gradByPoints.setZero();

        if (innerN == 0) return;

        // dE/d(boundary PVA) — direct effect through coefficients
        Eigen::MatrixX2d gradPVA(3 * (N + 1), 2);
        gradPVA.setZero();

        // Compute dE/dc for each piece, then map to dE/d(boundary PVA of that piece)
        for (int i = 0; i < N; i++) {
            const double Ti = T(i);
            const double Ti2 = Ti * Ti;
            const double Ti3 = Ti2 * Ti;
            const double Ti4 = Ti3 * Ti;
            const double Ti5 = Ti4 * Ti;

            const Eigen::Vector2d c3 = coeffs.row(6 * i + 3).transpose();
            const Eigen::Vector2d c4 = coeffs.row(6 * i + 4).transpose();
            const Eigen::Vector2d c5 = coeffs.row(6 * i + 5).transpose();

            // dE/dc3, dE/dc4, dE/dc5 for this piece
            Eigen::Vector2d dE_dc3 = 72.0 * c3 * Ti + 144.0 * c4 * Ti2
                                   + 240.0 * c5 * Ti3;
            Eigen::Vector2d dE_dc4 = 144.0 * c3 * Ti2 + 384.0 * c4 * Ti3
                                   + 720.0 * c5 * Ti4;
            Eigen::Vector2d dE_dc5 = 240.0 * c3 * Ti3 + 720.0 * c4 * Ti4
                                   + 1440.0 * c5 * Ti5;

            // Map dE/dc -> dE/d(boundary PVA) for piece i
            // Piece i connects knot i to knot i+1
            // The coefficient-to-boundary mapping Jacobian:
            mapCoeffGradToBoundary(i, dE_dc3, dE_dc4, dE_dc5, gradPVA);
        }

        // After the adjoint solve for optimal inner vel/acc, the gradient
        // of the inner waypoint positions includes both:
        // 1. Direct effect: dE/dp_k (position row of gradPVA)
        // 2. Indirect effect through optimal vel/acc change
        //
        // For the MINCO min-jerk problem, at the optimum the indirect effect
        // through optimal vel/acc is zero (they're at the minimum), so the
        // gradient w.r.t. waypoints is simply the position rows of gradPVA
        // plus the adjoint correction.

        // The adjoint correction accounts for the fact that changing p_k
        // shifts the optimal v_k, a_k via the tridiagonal system.
        // At the optimum: dE/dv_k = 0, dE/da_k = 0, so the total gradient
        // w.r.t. p_k is just the partial through the coefficients.

        // Solve the adjoint system for the tridiagonal M-matrix
        Eigen::MatrixX2d adjointGrad = solveAdjointForWaypoints(gradPVA);

        for (int i = 0; i < innerN; i++) {
            gradByPoints.col(i) = adjointGrad.row(i).transpose();
        }
    }

    /**
     * @brief Gradient of jerk energy w.r.t. segment durations T_i.
     *
     * dE/dT_i has two components:
     * 1. Direct: d/dT_i of the integral (upper limit change + coefficients depend on T)
     * 2. Indirect: coefficients depend on T through the M-matrix solve
     */
    inline void getGradTimes(Eigen::VectorXd &gradByTimes) const
    {
        gradByTimes.resize(N);
        gradByTimes.setZero();

        for (int i = 0; i < N; i++) {
            const double Ti = T(i);
            const double Ti2 = Ti * Ti;
            const double Ti3 = Ti2 * Ti;
            const double Ti4 = Ti3 * Ti;

            const Eigen::Vector2d c3 = coeffs.row(6 * i + 3).transpose();
            const Eigen::Vector2d c4 = coeffs.row(6 * i + 4).transpose();
            const Eigen::Vector2d c5 = coeffs.row(6 * i + 5).transpose();

            // Direct partial: d/dT_i [ Int_0^{T_i} ||p''(t)||^2 dt ]
            //   evaluated at t = T_i: ||p''(T_i)||^2
            Eigen::Vector2d jerk_at_Ti = 6.0 * c3 + 24.0 * c4 * Ti + 60.0 * c5 * Ti2;
            double direct = jerk_at_Ti.squaredNorm();

            // Coefficient change effect through boundary re-mapping
            // When T_i changes, the coefficient mapping changes.
            // For the quintic piece connecting (p0,v0,a0)->(p1,v1,a1) over duration T_i:
            // c0=p0, c1=v0, c2=a0/2 are independent of T_i
            // c3,c4,c5 depend on T_i through the boundary mapping
            double coeff_effect = computeTimeCoefficientGrad(i);

            gradByTimes(i) = direct + coeff_effect;
        }
    }

    /**
     * @brief Evaluate position at piece `piece`, parameter `t` ∈ [0, T_piece]
     */
    inline Eigen::Vector2d evaluate(int piece, double t) const
    {
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        return coeffs.row(6 * piece).transpose()
             + coeffs.row(6 * piece + 1).transpose() * t
             + coeffs.row(6 * piece + 2).transpose() * t2
             + coeffs.row(6 * piece + 3).transpose() * t3
             + coeffs.row(6 * piece + 4).transpose() * t4
             + coeffs.row(6 * piece + 5).transpose() * t5;
    }

    /**
     * @brief Evaluate velocity at piece `piece`, parameter `t`
     */
    inline Eigen::Vector2d evaluateVel(int piece, double t) const
    {
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        return coeffs.row(6 * piece + 1).transpose()
             + 2.0 * coeffs.row(6 * piece + 2).transpose() * t
             + 3.0 * coeffs.row(6 * piece + 3).transpose() * t2
             + 4.0 * coeffs.row(6 * piece + 4).transpose() * t3
             + 5.0 * coeffs.row(6 * piece + 5).transpose() * t4;
    }

    /**
     * @brief Evaluate acceleration at piece `piece`, parameter `t`
     */
    inline Eigen::Vector2d evaluateAcc(int piece, double t) const
    {
        const double t2 = t * t;
        const double t3 = t2 * t;
        return 2.0 * coeffs.row(6 * piece + 2).transpose()
             + 6.0 * coeffs.row(6 * piece + 3).transpose() * t
             + 12.0 * coeffs.row(6 * piece + 4).transpose() * t2
             + 20.0 * coeffs.row(6 * piece + 5).transpose() * t3;
    }

    /**
     * @brief Get max velocity magnitude across all pieces (sampled)
     */
    inline double getMaxVelocity(int samples_per_piece = 8) const
    {
        double maxVel = 0.0;
        for (int i = 0; i < N; i++) {
            for (int s = 0; s <= samples_per_piece; s++) {
                double t = T(i) * s / samples_per_piece;
                double vel = evaluateVel(i, t).norm();
                if (vel > maxVel) maxVel = vel;
            }
        }
        return maxVel;
    }

    /**
     * @brief Get max velocity per piece (sampled)
     */
    inline void getMaxVelocityPerPiece(Eigen::VectorXd &maxVels, int samples = 8) const
    {
        maxVels.resize(N);
        for (int i = 0; i < N; i++) {
            double mv = 0.0;
            for (int s = 0; s <= samples; s++) {
                double t = T(i) * s / samples;
                double vel = evaluateVel(i, t).norm();
                if (vel > mv) mv = vel;
            }
            maxVels(i) = mv;
        }
    }

    inline int getPieceNum() const { return N; }
    inline const Eigen::VectorXd& getDurations() const { return T; }
    inline const Eigen::MatrixX2d& getCoeffs() const { return coeffs; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    /**
     * @brief Compute quintic polynomial coefficients for each piece from
     *        boundary PVA values.
     *
     * For piece i connecting knot i (p0,v0,a0) to knot i+1 (p1,v1,a1)
     * over duration T_i:
     *   c0 = p0
     *   c1 = v0
     *   c2 = a0/2
     *   c3 = (20*(p1-p0) - (8*v1 + 12*v0)*T - (3*a0 - a1)*T^2) / (2*T^3)
     *   c4 = (30*(p0-p1) + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T^2) / (2*T^4)
     *   c5 = (12*(p1-p0) - 6*(v1 + v0)*T - (a0 - a1)*T^2) / (2*T^5)
     */
    inline void computeCoeffs()
    {
        for (int i = 0; i < N; i++) {
            const double Ti = T(i);
            const double Ti2 = Ti * Ti;
            const double Ti3 = Ti2 * Ti;
            const double Ti4 = Ti3 * Ti;
            const double Ti5 = Ti4 * Ti;

            const Eigen::Vector2d p0 = boundaryPVA.row(3 * i).transpose();
            const Eigen::Vector2d v0 = boundaryPVA.row(3 * i + 1).transpose();
            const Eigen::Vector2d a0 = boundaryPVA.row(3 * i + 2).transpose();
            const Eigen::Vector2d p1 = boundaryPVA.row(3 * (i + 1)).transpose();
            const Eigen::Vector2d v1 = boundaryPVA.row(3 * (i + 1) + 1).transpose();
            const Eigen::Vector2d a1 = boundaryPVA.row(3 * (i + 1) + 2).transpose();

            const Eigen::Vector2d dp = p1 - p0;

            coeffs.row(6 * i + 0) = p0.transpose();
            coeffs.row(6 * i + 1) = v0.transpose();
            coeffs.row(6 * i + 2) = (a0 / 2.0).transpose();
            coeffs.row(6 * i + 3) = ((20.0 * dp - (8.0 * v1 + 12.0 * v0) * Ti
                                      - (3.0 * a0 - a1) * Ti2) / (2.0 * Ti3)).transpose();
            coeffs.row(6 * i + 4) = ((30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * Ti
                                      + (3.0 * a0 - 2.0 * a1) * Ti2) / (2.0 * Ti4)).transpose();
            coeffs.row(6 * i + 5) = ((12.0 * dp - 6.0 * (v1 + v0) * Ti
                                      - (a0 - a1) * Ti2) / (2.0 * Ti5)).transpose();
        }
    }

    /**
     * @brief Solve for optimal inner velocities and accelerations
     *        that minimize jerk energy, given fixed positions and durations.
     *
     * The optimality conditions form a block-tridiagonal system in the unknown
     * (v_k, a_k) at each interior knot k = 1..N-1.
     *
     * Hessian and gradient blocks are derived from the Jacobian-Hessian
     * factorization: for each quintic piece with coefficients c3,c4,c5 that
     * depend linearly on boundary (v,a), the energy E = c^T Q c yields
     * H_boundary = J^T (2Q) J and g_boundary = J^T (2Q) c_fixed.
     *
     * Verified Hessian blocks (per piece, per axis):
     *   H_RR (right-end v1,a1):  [[384/T^3, -72/T^2], [-72/T^2, 18/T]]
     *   H_LL (left-end  v0,a0):  [[384/T^3,  72/T^2], [ 72/T^2, 18/T]]
     *   H_LR (left->right coupling): [[336/T^3, -48/T^2], [48/T^2, -6/T]]
     *
     * We solve per-axis using Thomas algorithm (block LU for tridiagonal).
     */
    inline void solveOptimalVelAcc(const Eigen::Ref<const Eigen::Matrix2Xd> &innerPs)
    {
        const int innerN = N - 1;
        if (innerN == 0) return;

        // Solve for each axis independently (x and y)
        for (int axis = 0; axis < 2; axis++) {
            // Block-tridiagonal: lower[k]*x_{k-1} + diag[k]*x_k + upper[k]*x_{k+1} = rhs[k]
            // x_k = (v_{k+1}, a_{k+1}) at interior knot k (0-indexed)
            std::vector<Eigen::Matrix2d> diag(innerN);
            std::vector<Eigen::Matrix2d> upper(innerN);
            std::vector<Eigen::Matrix2d> lower(innerN);
            std::vector<Eigen::Vector2d> rhs(innerN);

            for (int k = 0; k < innerN; k++) {
                // Piece k (left of interior knot k) has duration TL
                // Piece k+1 (right of interior knot k) has duration TR
                const double TL = T(k);
                const double TR = T(k + 1);
                const double TL2 = TL * TL;
                const double TR2 = TR * TR;
                const double TL3 = TL2 * TL;
                const double TR3 = TR2 * TR;
                const double TL4 = TL3 * TL;
                const double TR4 = TR3 * TR;

                // Diagonal: H_RR(TL) + H_LL(TR)
                diag[k](0, 0) = 384.0 / TL3 + 384.0 / TR3;
                diag[k](0, 1) = -72.0 / TL2 + 72.0 / TR2;
                diag[k](1, 0) = -72.0 / TL2 + 72.0 / TR2;
                diag[k](1, 1) = 18.0 / TL + 18.0 / TR;

                // Lower: H_LR(TL)^T — coupling k to k-1 through piece k
                if (k > 0) {
                    lower[k](0, 0) = 336.0 / TL3;
                    lower[k](0, 1) = 48.0 / TL2;
                    lower[k](1, 0) = -48.0 / TL2;
                    lower[k](1, 1) = -6.0 / TL;
                } else {
                    lower[k].setZero();
                }

                // Upper: H_LR(TR) — coupling k to k+1 through piece k+1
                if (k < innerN - 1) {
                    upper[k](0, 0) = 336.0 / TR3;
                    upper[k](0, 1) = -48.0 / TR2;
                    upper[k](1, 0) = 48.0 / TR2;
                    upper[k](1, 1) = -6.0 / TR;
                } else {
                    upper[k].setZero();
                }

                // RHS: negative gradient from fixed terms (positions + boundary v/a)
                double pL = boundaryPVA(3 * k, axis);       // p_k (left of left piece)
                double pM = innerPs(axis, k);                // p_{k+1} (this knot)
                double pR;
                if (k + 1 < innerN) {
                    pR = innerPs(axis, k + 1);               // p_{k+2}
                } else {
                    pR = boundaryPVA(3 * N, axis);            // tail position
                }

                double dpL = pM - pL;  // left piece displacement
                double dpR = pR - pM;  // right piece displacement

                // Position gradient: dE/dv from dp (verified via Jacobian-Hessian)
                // Left piece (right end): dE_L/dv1 = -720*dpL/TL^4
                // Left piece (right end): dE_L/da1 =  120*dpL/TL^3
                double gv_L = -720.0 * dpL / TL4;
                double ga_L =  120.0 * dpL / TL3;

                // Right piece (left end): dE_R/dv0 = -720*dpR/TR^4
                // Right piece (left end): dE_R/da0 = -120*dpR/TR^3
                double gv_R = -720.0 * dpR / TR4;
                double ga_R = -120.0 * dpR / TR3;

                // Head boundary (k=0): contribution from fixed headV, headA
                // via cross-Hessian H_LR(TL)^T applied to (headV, headA)
                double vL0 = boundaryPVA(3 * k + 1, axis);   // headV if k=0, else 0
                double aL0 = boundaryPVA(3 * k + 2, axis);   // headA if k=0, else 0
                gv_L +=  336.0 * vL0 / TL3 + 48.0 * aL0 / TL2;
                ga_L += -48.0 * vL0 / TL2  - 6.0  * aL0 / TL;

                // Tail boundary (k=innerN-1): contribution from fixed tailV, tailA
                // via cross-Hessian H_LR(TR) applied to (tailV, tailA)
                double vR1 = (k + 1 < innerN) ? 0.0 : boundaryPVA(3 * N + 1, axis);
                double aR1 = (k + 1 < innerN) ? 0.0 : boundaryPVA(3 * N + 2, axis);
                gv_R +=  336.0 * vR1 / TR3 - 48.0 * aR1 / TR2;
                ga_R +=   48.0 * vR1 / TR2 -  6.0 * aR1 / TR;

                // System: H*x = -g, so rhs = -g
                rhs[k](0) = -(gv_L + gv_R);
                rhs[k](1) = -(ga_L + ga_R);
            }

            // Thomas algorithm: forward elimination
            std::vector<Eigen::Matrix2d> diagMod(innerN);
            std::vector<Eigen::Vector2d> rhsMod(innerN);
            diagMod[0] = diag[0];
            rhsMod[0] = rhs[0];

            for (int k = 1; k < innerN; k++) {
                Eigen::Matrix2d Linv = diagMod[k - 1].inverse();
                Eigen::Matrix2d factor = lower[k] * Linv;
                diagMod[k] = diag[k] - factor * upper[k - 1];
                rhsMod[k] = rhs[k] - factor * rhsMod[k - 1];
            }

            // Back substitution
            std::vector<Eigen::Vector2d> sol(innerN);
            sol[innerN - 1] = diagMod[innerN - 1].inverse() * rhsMod[innerN - 1];
            for (int k = innerN - 2; k >= 0; k--) {
                sol[k] = diagMod[k].inverse() * (rhsMod[k] - upper[k] * sol[k + 1]);
            }

            // Store: sol[k] = (v_{k+1}, a_{k+1}) for interior knot k+1
            for (int k = 0; k < innerN; k++) {
                boundaryPVA(3 * (k + 1) + 1, axis) = sol[k](0);
                boundaryPVA(3 * (k + 1) + 2, axis) = sol[k](1);
            }
        }
    }

    /**
     * @brief Map dE/d(c3,c4,c5) of piece i to dE/d(boundary PVA) accumulator.
     *
     * Uses the Jacobian of the coefficient mapping:
     *   c3 = (20*dp - (8*v1+12*v0)*T - (3*a0-a1)*T^2) / (2*T^3)
     *   c4 = (30*(p0-p1) + (14*v1+16*v0)*T + (3*a0-2*a1)*T^2) / (2*T^4)
     *   c5 = (12*dp - 6*(v1+v0)*T - (a0-a1)*T^2) / (2*T^5)
     */
    inline void mapCoeffGradToBoundary(int i,
                                       const Eigen::Vector2d &dE_dc3,
                                       const Eigen::Vector2d &dE_dc4,
                                       const Eigen::Vector2d &dE_dc5,
                                       Eigen::MatrixX2d &gradPVA) const
    {
        const double Ti = T(i);
        const double Ti2 = Ti * Ti;
        const double Ti3 = Ti2 * Ti;
        const double Ti4 = Ti3 * Ti;
        const double Ti5 = Ti4 * Ti;

        // dc/dp0, dc/dp1
        Eigen::Vector2d dcp0 = -10.0 / Ti3 * dE_dc3 + 15.0 / Ti4 * dE_dc4 - 6.0 / Ti5 * dE_dc5;
        Eigen::Vector2d dcp1 =  10.0 / Ti3 * dE_dc3 - 15.0 / Ti4 * dE_dc4 + 6.0 / Ti5 * dE_dc5;

        // dc/dv0, dc/dv1
        Eigen::Vector2d dcv0 = -6.0 / Ti2 * dE_dc3 + 8.0 / Ti3 * dE_dc4 - 3.0 / Ti4 * dE_dc5;
        Eigen::Vector2d dcv1 = -4.0 / Ti2 * dE_dc3 + 7.0 / Ti3 * dE_dc4 - 3.0 / Ti4 * dE_dc5;

        // dc/da0, dc/da1
        Eigen::Vector2d dca0 = -3.0 / (2.0 * Ti) * dE_dc3 + 3.0 / (2.0 * Ti2) * dE_dc4
                             - 1.0 / (2.0 * Ti3) * dE_dc5;
        Eigen::Vector2d dca1 =  1.0 / (2.0 * Ti) * dE_dc3 - 1.0 / Ti2 * dE_dc4
                             + 1.0 / (2.0 * Ti3) * dE_dc5;

        // Accumulate into boundary PVA gradient
        gradPVA.row(3 * i)     += dcp0.transpose();
        gradPVA.row(3 * i + 1) += dcv0.transpose();
        gradPVA.row(3 * i + 2) += dca0.transpose();
        gradPVA.row(3 * (i + 1))     += dcp1.transpose();
        gradPVA.row(3 * (i + 1) + 1) += dcv1.transpose();
        gradPVA.row(3 * (i + 1) + 2) += dca1.transpose();
    }

    /**
     * @brief Solve the adjoint of the tridiagonal system to propagate gradients
     *        from boundary PVA back to waypoint positions.
     *
     * At the optimum, dE/dv_k = 0 and dE/da_k = 0 for interior knots.
     * The total gradient w.r.t. p_k is:
     *   dE/dp_k = dE/dp_k + (dv_star/dp_k)^T * 0 + (da_star/dp_k)^T * 0
     *           = dE/dp_k  (from gradPVA position rows)
     *
     * However, we also need to account for how changing p_k affects
     * the RHS of the tridiagonal system, which changes v_star, a_star.
     * By the envelope theorem at the optimum, the indirect effect is zero
     * ONLY if the optimality conditions are exactly satisfied.
     * In practice we solve exactly, so this holds.
     */
    inline Eigen::MatrixX2d solveAdjointForWaypoints(const Eigen::MatrixX2d &gradPVA) const
    {
        const int innerN = N - 1;
        Eigen::MatrixX2d result(innerN, 2);

        for (int k = 0; k < innerN; k++) {
            // The gradient of E w.r.t. interior position p_{k+1}
            // is simply the position row of gradPVA at knot k+1
            // (by envelope theorem, since vel/acc are at their optimal values)
            result.row(k) = gradPVA.row(3 * (k + 1));
        }

        return result;
    }

    /**
     * @brief Compute the time-gradient contribution from coefficient changes.
     *
     * When T_i changes, the coefficients c3,c4,c5 change (through the boundary
     * mapping formulas), and the optimal v/a at interior knots also change.
     *
     * Total: dE/dT_i = ||p''(T_i)||^2 + Sum (dE/dc_j) * (dc_j/dT_i)
     *        + (dv_star/dT) * dE/dv + (da_star/dT) * dE/da
     *
     * At optimum, last two terms vanish (envelope theorem).
     * The coefficient derivative terms: dc3/dT_i, dc4/dT_i, dc5/dT_i
     */
    inline double computeTimeCoefficientGrad(int i) const
    {
        const double Ti = T(i);
        const double Ti2 = Ti * Ti;
        const double Ti3 = Ti2 * Ti;
        const double Ti4 = Ti3 * Ti;
        const double Ti5 = Ti4 * Ti;

        const Eigen::Vector2d p0 = boundaryPVA.row(3 * i).transpose();
        const Eigen::Vector2d v0 = boundaryPVA.row(3 * i + 1).transpose();
        const Eigen::Vector2d a0 = boundaryPVA.row(3 * i + 2).transpose();
        const Eigen::Vector2d p1 = boundaryPVA.row(3 * (i + 1)).transpose();
        const Eigen::Vector2d v1 = boundaryPVA.row(3 * (i + 1) + 1).transpose();
        const Eigen::Vector2d a1 = boundaryPVA.row(3 * (i + 1) + 2).transpose();

        const Eigen::Vector2d dp = p1 - p0;

        const Eigen::Vector2d c3 = coeffs.row(6 * i + 3).transpose();
        const Eigen::Vector2d c4 = coeffs.row(6 * i + 4).transpose();
        const Eigen::Vector2d c5 = coeffs.row(6 * i + 5).transpose();

        // dc3/dT = d/dT[(20*dp - (8v1+12v0)T - (3a0-a1)T^2) / (2T^3)]
        //        = [-(8v1+12v0)*2T^3 - 6T^2*(20*dp-(8v1+12v0)T-(3a0-a1)T^2)] / (4T^6)
        //        = [-(8v1+12v0) / (2T^3)] + [-3 * c3 / T] + [(3a0-a1) / (2T^2) - 3*c3/T ... ]
        // Simplified using c3 itself:
        // dc3/dT = (-(8v1+12v0)/(2T^3)) + (-3*c3/T) + (-(3a0-a1)*2T/(2T^3*2))
        // Let's compute numerically-safely using the full derivative:
        Eigen::Vector2d num3 = 20.0 * dp - (8.0 * v1 + 12.0 * v0) * Ti - (3.0 * a0 - a1) * Ti2;
        Eigen::Vector2d dnum3_dT = -(8.0 * v1 + 12.0 * v0) - 2.0 * (3.0 * a0 - a1) * Ti;
        Eigen::Vector2d dc3_dT = (dnum3_dT * 2.0 * Ti3 - num3 * 6.0 * Ti2) / (4.0 * Ti3 * Ti3);

        Eigen::Vector2d num4 = 30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * Ti
                             + (3.0 * a0 - 2.0 * a1) * Ti2;
        Eigen::Vector2d dnum4_dT = (14.0 * v1 + 16.0 * v0) + 2.0 * (3.0 * a0 - 2.0 * a1) * Ti;
        Eigen::Vector2d dc4_dT = (dnum4_dT * 2.0 * Ti4 - num4 * 8.0 * Ti3) / (4.0 * Ti4 * Ti4);

        Eigen::Vector2d num5 = 12.0 * dp - 6.0 * (v1 + v0) * Ti - (a0 - a1) * Ti2;
        Eigen::Vector2d dnum5_dT = -6.0 * (v1 + v0) - 2.0 * (a0 - a1) * Ti;
        Eigen::Vector2d dc5_dT = (dnum5_dT * 2.0 * Ti5 - num5 * 10.0 * Ti4) / (4.0 * Ti5 * Ti5);

        // dE/dc3, dE/dc4, dE/dc5 (same as in getGradWaypoints)
        Eigen::Vector2d dE_dc3 = 72.0 * c3 * Ti + 144.0 * c4 * Ti2 + 240.0 * c5 * Ti3;
        Eigen::Vector2d dE_dc4 = 144.0 * c3 * Ti2 + 384.0 * c4 * Ti3 + 720.0 * c5 * Ti4;
        Eigen::Vector2d dE_dc5 = 240.0 * c3 * Ti3 + 720.0 * c4 * Ti4 + 1440.0 * c5 * Ti5;

        return dE_dc3.dot(dc3_dT) + dE_dc4.dot(dc4_dT) + dE_dc5.dot(dc5_dT);
    }
};

#endif // SENTRY_PLANNING_MINCO_TRAJECTORY
