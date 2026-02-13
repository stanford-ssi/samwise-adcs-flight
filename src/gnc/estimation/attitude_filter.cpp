/**
 * @author Lundeen Cahilly, Niklas Vainio, Chen Li
 * @brief This file implements an attitude EKF
 * using modified Rodrigues parameters (MRPs)
 * @date 2025-10-16
 *
 * Based on this paper:
 * https://ntrs.nasa.gov/api/citations/19960035754/downloads/19960035754.pdf
 * General EKF info from
 * https://stanfordasl.github.io/PoRA-I/aa174a_aut2526/resources/PoRA.pdf
 */

#include "attitude_filter.h"
#include "gnc/utils/matrix_utils.h"
#include "gnc/utils/utils.h"
#include "linalg.h"
#include "macros.h"
#include "params.h"
#include "pico/time.h"

// ========================================================================
//      COVARIANCE MATRICES
// ========================================================================
// Process noise covariance matrix
static float Q[6 * 6] = {
    IMU_GYRO_VARIANCE,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    IMU_GYRO_VARIANCE,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    IMU_GYRO_VARIANCE,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    IMU_DRIFT_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    IMU_DRIFT_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    IMU_DRIFT_VARIANCE,
};

// Measurement noise covariance matrix
static float R_sun[3 * 3] = {SUN_VECTOR_VARIANCE, 0.0f, 0.0f, 0.0f,
                             SUN_VECTOR_VARIANCE, 0.0f, 0.0f, 0.0f,
                             SUN_VECTOR_VARIANCE};
static float R_mag[3 * 3] = {MAGNETOMETER_VARIANCE, 0.0f, 0.0f, 0.0f,
                             MAGNETOMETER_VARIANCE, 0.0f, 0.0f, 0.0f,
                             MAGNETOMETER_VARIANCE};

// ========================================================================
//      JACOBIAN AND STATE TRANSITION MATRIX UTILITIES
// ========================================================================

/**
 * @brief Compute the state derivative x_dot = f(x, w)
 * where x = [p; b] (MRP and gyro bias) and w is the angular velocity
 * measurement from the gyroscope. See equation 20a in the MRP EKF paper.
 *
 * @param x_dot Output state derivative (6-vector)
 * @param x Current state (6-vector)
 * @param w Measured angular velocity from gyroscope (3-vector)
 */
void compute_x_dot(float *x_dot, const float *x, const float3 w)
{
    // Extract state components
    float3 p = {x[0], x[1], x[2]}; // MRP
    float3 b = {x[3], x[4], x[5]}; // Angular velocity bias from gyro

    // Reconstruct MRP kinematics equation 20(a)
    // f(p) = 0.5 * [0.5 * (1 - p^T p) * I + cross_matrix(p) + outerprod(p, p)]
    // * (w - b)
    float p_dot_p = dot(p, p);
    float scalar_coeff = 0.5f * (1.0f - p_dot_p);

    // Identity component: 0.5 * scalar_coeff * I
    float3x3 identity_term = {{0.5f * scalar_coeff, 0.0f, 0.0f},
                              {0.0f, 0.5f * scalar_coeff, 0.0f},
                              {0.0f, 0.0f, 0.5f * scalar_coeff}};

    float3x3 cross_term = 0.5f * cross_matrix(p);
    float3x3 outer_term = 0.5f * outerprod(p, p);
    float3x3 G = identity_term + cross_term + outer_term;

    float3 w_minus_b = w - b;
    float3 p_dot = mul(G, w_minus_b);

    // Populate output: [p_dot; b_dot] where b_dot = 0
    x_dot[0] = p_dot.x;
    x_dot[1] = p_dot.y;
    x_dot[2] = p_dot.z;
    x_dot[3] = 0.0f; // b_dot = 0 (assume constant bias model)
    x_dot[4] = 0.0f;
    x_dot[5] = 0.0f;
}

void compute_B_B(float *B_B, const float *x, const float3 B_I)
{
    // Reconstruct equation (10)
    // Extract MRP from state
    float3 p = {x[0], x[1], x[2]};

    // Rotate inertial reference vector to body frame
    float3 z = mul(mrp_to_dcm(p), B_I);

    // Output expected measurement in body frame
    B_B[0] = z.x;
    B_B[1] = z.y;
    B_B[2] = z.z;
}

void compute_F(float *F, const float *x, const float3 w)
{
    // Compute the Jacobian F = df/dx with respect to state x = [p; b]
    float3 p = {x[0], x[1], x[2]};
    float3 b = {x[3], x[4], x[5]};
    float3 w_hat = w - b;

    // Reconstruct equations (25a)
    // df_dp = 0.5 * (outer(p, w_hat) - outer(w_hat, p) - cross_matrix(w_hat) +
    // dot(w_hat, p) * I)
    float p_dot_w_hat = dot(p, w_hat);
    float3x3 outer_p_w_hat = outerprod(p, w_hat);
    float3x3 cross_w_hat = cross_matrix(w_hat);
    float3x3 outer_w_hat_p = outerprod(w_hat, p);

    float3x3 df_dp = 0.5f * (outer_p_w_hat - outer_w_hat_p - cross_w_hat +
                             p_dot_w_hat * identity3x3);

    // Reconstruct equation (25b)
    // df_db = -0.5 * (0.5 * (1 - dot(p, p)) * I + cross_matrix(p) +
    // outerprod(p, p))
    float3x3 cross_p = cross_matrix(p);
    float3x3 outer_p_p = outerprod(p, p);
    float p_dot_p = dot(p, p);

    float3x3 df_db =
        -0.5f * (0.5f * (1 - p_dot_p) * identity3x3 + cross_p + outer_p_p);

    // Reconstruct equation (22a)
    // F = [df_dp  df_db; 0_3x3  0_3x3]
    // Top-left 3x3 block: df_dp
    F[0] = df_dp[0][0];
    F[1] = df_dp[0][1];
    F[2] = df_dp[0][2];
    F[6] = df_dp[1][0];
    F[7] = df_dp[1][1];
    F[8] = df_dp[1][2];
    F[12] = df_dp[2][0];
    F[13] = df_dp[2][1];
    F[14] = df_dp[2][2];
    // Top-right 3x3 block: df_db
    F[3] = df_db[0][0];
    F[4] = df_db[0][1];
    F[5] = df_db[0][2];
    F[9] = df_db[1][0];
    F[10] = df_db[1][1];
    F[11] = df_db[1][2];
    F[15] = df_db[2][0];
    F[16] = df_db[2][1];
    F[17] = df_db[2][2];
    // Bottom-left and bottom-right 3x3 blocks: zeros
    for (int i = 18; i < 36; i++)
    {
        F[i] = 0.0f;
    }
}

void compute_G(float *G, const float *x)
{
    // Compute jacobian of process noise G = df/dw
    float3 p = {x[0], x[1], x[2]};

    // Reconstruct equation (23a)
    float3x3 outer_p_p = outerprod(p, p);
    float p_dot_p = dot(p, p);
    float3x3 p_cross = cross_matrix(p);
    float3x3 G11 =
        -0.5f * (0.5f * (1 - p_dot_p) * identity3x3 + p_cross + outer_p_p);

    // Reconstruct equation (22b)
    // F = [G11  0_3x3; 0_3x3  identity]
    // Top-left 3x3 block: G11
    G[0] = G11[0][0];
    G[1] = G11[0][1];
    G[2] = G11[0][2];
    G[6] = G11[1][0];
    G[7] = G11[1][1];
    G[8] = G11[1][2];
    G[12] = G11[2][0];
    G[13] = G11[2][1];
    G[14] = G11[2][2];
    // Top-right block: 0_3x3
    G[3] = 0.0f;
    G[4] = 0.0f;
    G[5] = 0.0f;
    G[9] = 0.0f;
    G[10] = 0.0f;
    G[11] = 0.0f;
    G[15] = 0.0f;
    G[16] = 0.0f;
    G[17] = 0.0f;
    // Bottom-left block: 0_3x3
    G[18] = 0.0f;
    G[19] = 0.0f;
    G[20] = 0.0f;
    G[24] = 0.0f;
    G[25] = 0.0f;
    G[26] = 0.0f;
    G[30] = 0.0f;
    G[31] = 0.0f;
    G[32] = 0.0f;
    // Bottom-right block: identity
    G[21] = 1.0f;
    G[22] = 0.0f;
    G[23] = 0.0f;
    G[27] = 0.0f;
    G[28] = 1.0f;
    G[29] = 0.0f;
    G[33] = 0.0f;
    G[34] = 0.0f;
    G[35] = 1.0f;
}

void compute_H(float *H, const float *x, const float3 B_I)
{
    // Compute jacobian of measurement model with respect to the state
    float3 p = {x[0], x[1], x[2]};
    float3x3 A = mrp_to_dcm(p);

    // Reconstruct equation (39)
    float p_dot_p = dot(p, p);
    float3x3 outer_p_p = outerprod(p, p);
    float3x3 A_B_I_cross = cross_matrix(mul(A, B_I));
    float3x3 p_cross = cross_matrix(p);
    float3x3 bracket =
        (1.0f - p_dot_p) * identity3x3 - 2.0f * p_cross + 2.0f * outer_p_p;
    float scalar = 4.0f / ((1 + p_dot_p) * (1 + p_dot_p));
    float3x3 inverse_bracket;
    mat_inverse(&bracket.x.x, &inverse_bracket.x.x, 3);
    float3x3 L;
    mat_mul_square(&A_B_I_cross.x.x, &inverse_bracket.x.x, &L.x.x, 3);

    // Reconstruct eqn (30)
    // H = [L  0_3x3]
    // Left 3x3 block: L
    H[0] = L[0][0] * scalar;
    H[1] = L[0][1] * scalar;
    H[2] = L[0][2] * scalar;
    H[6] = L[1][0] * scalar;
    H[7] = L[1][1] * scalar;
    H[8] = L[1][2] * scalar;
    H[12] = L[2][0] * scalar;
    H[13] = L[2][1] * scalar;
    H[14] = L[2][2] * scalar;
    // Right 3x3 block: 0
    H[3] = 0.0f;
    H[4] = 0.0f;
    H[5] = 0.0f;
    H[9] = 0.0f;
    H[10] = 0.0f;
    H[11] = 0.0f;
    H[15] = 0.0f;
    H[16] = 0.0f;
    H[17] = 0.0f;
}

// ========================================================================
//      ATTITUDE FILTER STEPS
// ========================================================================

/**
 * @brief Initialize the attitude filter state and covariance
 *
 * @param slate Pointer to the ADCS slate structure
 */
void attitude_filter_init(slate_t *slate)
{
    // Set MRP to identity
    slate->p_eci_to_body = float3(0.0f, 0.0f, 0.0f);
    slate->q_eci_to_body = quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    slate->b_gyro_drift = float3(0.0f, 0.0f, 0.0f);
    // Fill attitude covariance matrix P
    for (int i = 0; i < 6 * 6; i++)
    {
        if (i % 7 == 0 and i < 18)
        {
            slate->P_attitude[i] =
                1.0f; // initial variance of 1.0 on diagonal for MRP
        }
        else if (i % 7 == 0 and i >= 18)
        {
            slate->P_attitude[i] =
                IMU_GYRO_VARIANCE; // initial variance on diagonal for gyro bias
        }
        else
        {
            slate->P_attitude[i] = 0.0f; // rest zeros
        }
    }
    slate->P_attitude_log_frobenius = mat_log_frobenius(slate->P_attitude, 6);
    slate->af_is_initialized = true;
    slate->af_init_count++;
    LOG_INFO("Initialized attitude filter! (%d times so far)",
             slate->af_init_count);
}

/**
 * @brief Propagate the attitude filter state and covariance
 * using Euler integration over the time step since last propagate.
 *
 * @param slate Pointer to the ADCS slate structure
 */
void attitude_filter_propagate(slate_t *slate)
{
    // Get dt since last propagate
    absolute_time_t current_time = get_absolute_time();
    if (!slate->af_last_propagate_time)
    {
        // If first time propagating, just set last run time and return
        slate->af_last_propagate_time = get_absolute_time();
        return;
    }
    float dt =
        absolute_time_diff_us(slate->af_last_propagate_time, current_time) *
        1e-6f; // Convert to seconds
    slate->af_last_propagate_time = current_time;

    // Grab state vector and covariance from slate
    float x[6] = {
        slate->p_eci_to_body[0], slate->p_eci_to_body[1],
        slate->p_eci_to_body[2], slate->b_gyro_drift[0],
        slate->b_gyro_drift[1],  slate->b_gyro_drift[2],
    };

    // Propagate state from equation (40a) using RK4 integration
    // x_dot = f(x,t)
    // x_new = x + (k1 + 2*k2 + 2*k3 + k4) / 6
    float3 w = slate->w_body;

    // RK4 integration
    float k1[6], k2[6], k3[6], k4[6];
    float x_temp[6];

    // k1 = f(x, t)
    compute_x_dot(k1, x, w);

    // k2 = f(x + dt/2 * k1, t + dt/2)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k1[i];
    }
    compute_x_dot(k2, x_temp, w);

    // k3 = f(x + dt/2 * k2, t + dt/2)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k2[i];
    }
    compute_x_dot(k3, x_temp, w);

    // k4 = f(x + dt * k3, t + dt)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + dt * k3[i];
    }
    compute_x_dot(k4, x_temp, w);

    // x_new = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    float x_new[6];
    for (int i = 0; i < 6; i++)
    {
        x_new[i] =
            x[i] + (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }
    // Write propagated state back to slate
    float3 p_eci_to_body = float3(x_new[0], x_new[1], x_new[2]);
    slate->p_eci_to_body = mrp_wrap_shadow_set(
        p_eci_to_body); // Wrap MRP to avoid it blowing up (IMPORTANT!)
    slate->q_eci_to_body = mrp_to_quat(slate->p_eci_to_body);
    slate->b_gyro_drift = float3(x_new[3], x_new[4], x_new[5]);

    // Propagate covariance from equation (40b)
    // P_dot = F @ P + P @ F^T + G @ Q @ G^T
    // Note: paper has an error in equation (40b),
    // where the term P^T @ F should be P @ F^T.
    float F[6 * 6];
    compute_F(F, x, w);
    float G[6 * 6];
    compute_G(G, x);

    float F_T[6 * 6];
    float G_T[6 * 6];
    float FP[6 * 6];
    float PF_T[6 * 6];
    float GQ[6 * 6];
    float GQG_T[6 * 6];
    mat_transpose(F, F_T, 6, 6);
    mat_transpose(G, G_T, 6, 6);
    mat_mul_square(F, slate->P_attitude, FP, 6);
    mat_mul_square(slate->P_attitude, F_T, PF_T, 6);
    mat_mul_square(G, Q, GQ, 6);
    mat_mul_square(GQ, G_T, GQG_T, 6);

    float FP_plus_PF_T[6 * 6];
    mat_add(FP, PF_T, FP_plus_PF_T, 6, 6);

    float P_dot[6 * 6];
    mat_add(FP_plus_PF_T, GQG_T, P_dot, 6, 6);

    // Propagate covariance matrix P_new = P + dt * P_dot
    for (int i = 0; i < 36; i++)
    {
        slate->P_attitude[i] = slate->P_attitude[i] + dt * P_dot[i];
    }
    // Update log frobenius norm of attitude covariance matrix
    slate->P_attitude_log_frobenius = mat_log_frobenius(slate->P_attitude, 6);

    // Check for NaN in state or covariance and reinitialize if detected
    if (std::isnan(slate->p_eci_to_body[0]) ||
        std::isnan(slate->p_eci_to_body[1]) ||
        std::isnan(slate->p_eci_to_body[2]) ||
        std::isnan(slate->q_eci_to_body[0]) ||
        std::isnan(slate->q_eci_to_body[1]) ||
        std::isnan(slate->q_eci_to_body[2]) ||
        std::isnan(slate->q_eci_to_body[3]) ||
        std::isnan(slate->b_gyro_drift[0]) ||
        std::isnan(slate->b_gyro_drift[1]) ||
        std::isnan(slate->b_gyro_drift[2]) ||
        std::isnan(slate->P_attitude_log_frobenius))
    {
        LOG_ERROR("[ekf] NaN detected in propagate!");
        LOG_ERROR("[ekf] Final state: p=[%.6f,%.6f,%.6f], b_drift=[%.6f,%.6f,%.6f]",
                  slate->p_eci_to_body[0], slate->p_eci_to_body[1], slate->p_eci_to_body[2],
                  slate->b_gyro_drift[0], slate->b_gyro_drift[1], slate->b_gyro_drift[2]);
        float mrp_norm_error = length(slate->p_eci_to_body);
        LOG_ERROR("[ekf] MRP norm = %.6f, P_log_frob = %.6f", mrp_norm_error, slate->P_attitude_log_frobenius);
        LOG_ERROR("[ekf] Reinitializing filter...");
        attitude_filter_init(slate);
        return;
    }

    // LOG_DEBUG("[ekf] q_eci_to_body = [%.6f, %.6f, %.6f, %.6f]",
    //           slate->q_eci_to_body[0], slate->q_eci_to_body[1],
    //           slate->q_eci_to_body[2], slate->q_eci_to_body[3]);
    // LOG_DEBUG("[ekf] b_gyro_drift = [%.6f, %.6f, %.6f]", slate->b_gyro_drift[0],
    //           slate->b_gyro_drift[1], slate->b_gyro_drift[2]);
    // LOG_DEBUG("[ekf] P_attitude_log_frobenius = %.6f",
    //           slate->P_attitude_log_frobenius);
}

/**
 * @brief Update the attitude filter state and covariance
 * using a measurement from either the sun sensor or magnetometer.
 *
 * @param slate Pointer to the ADCS slate structure
 * @param sensor_type 'S' for sun sensor, 'M' for magnetometer
 */
void attitude_filter_update(slate_t *slate, char sensor_type)
{
    // Update filter given one measurement from sun sensor OR magnetometer
    // TODO: figure out how to tell the filter which measurement was updated
    // this should likely be done in the task and passed via slate
    float x[6] = {
        slate->p_eci_to_body[0], slate->p_eci_to_body[1],
        slate->p_eci_to_body[2], slate->b_gyro_drift[0],
        slate->b_gyro_drift[1],  slate->b_gyro_drift[2],
    };
    if (sensor_type != 'S' && sensor_type != 'M')
    {
        LOG_ERROR("Attitude filter update called with invalid sensor type!");
        return;
    }
    float3 B_B;     // measured vector in body frame
    float3 B_I;     // reference vector in inertial frame
    float R[3 * 3]; // measurement noise covariance
    // Sun sensor update
    if (sensor_type == 'S')
    {
        B_B = slate->sun_vector_body;
        B_I = slate->sun_vector_eci;
        for (int i = 0; i < 3 * 3; i++)
        {
            R[i] = R_sun[i];
        }
    }
    // Magnetometer update
    else if (sensor_type == 'M')
    {
        B_B = slate->b_body;
        B_I = slate->b_eci;
        for (int i = 0; i < 3 * 3; i++)
        {
            R[i] = R_mag[i];
        }
    }
    // Compute Kalman gain (40e)
    float K[6 * 3];
    float H[3 * 6];
    float H_T[6 * 3];
    float PH_T[6 * 3];
    float HPH_T[3 * 3];
    float HPH_T_plus_R[3 * 3];
    float HPH_T_plus_R_inv[3 * 3];
    compute_H(H, x, B_I);
    mat_transpose(H, H_T, 3, 6);
    mat_mul(slate->P_attitude, H_T, PH_T, 6, 6, 3);
    mat_mul(H, PH_T, HPH_T, 3, 6, 3);
    mat_add(HPH_T, R, HPH_T_plus_R, 3, 3);
    mat_inverse(HPH_T_plus_R, HPH_T_plus_R_inv, 3);
    mat_mul(PH_T, HPH_T_plus_R_inv, K, 6, 3, 3);

    // Update state (40c)
    float B_B_expected[3];
    compute_B_B(B_B_expected, x, B_I);
    float innovation[3] = {
        B_B.x - B_B_expected[0],
        B_B.y - B_B_expected[1],
        B_B.z - B_B_expected[2],
    };
    float K_innovation[6];
    float x_new[6];
    mat_mul(K, innovation, K_innovation, 6, 3, 1);
    mat_add(x, K_innovation, x_new, 6, 1);

    // Write updated state back to slate
    float3 p_eci_to_body = float3(x_new[0], x_new[1], x_new[2]);
    slate->p_eci_to_body = mrp_wrap_shadow_set(
        p_eci_to_body); // Wrap MRP to avoid it blowing up (IMPORTANT!)
    slate->q_eci_to_body = mrp_to_quat(slate->p_eci_to_body);
    slate->b_gyro_drift = float3(x_new[3], x_new[4], x_new[5]);

    // Update covariance using Joseph form (more numerically stable)
    // P = (I - K*H)*P*(I - K*H)' + K*R*K'
    float P_new[6 * 6];
    float KH[6 * 6];
    float I_minus_KH[6 * 6];
    float I_minus_KH_T[6 * 6];
    float I_minus_KH_P[6 * 6];
    float I_minus_KH_P_I_minus_KH_T[6 * 6];
    float K_T[3 * 6];
    float KR[6 * 3];
    float KRK_T[6 * 6];

    // Compute (I - K*H)
    mat_mul(K, H, KH, 6, 3, 6);
    mat_sub(identity6x6, KH, I_minus_KH, 6, 6);

    // Compute (I - K*H)*P*(I - K*H)'
    mat_transpose(I_minus_KH, I_minus_KH_T, 6, 6);
    mat_mul(I_minus_KH, slate->P_attitude, I_minus_KH_P, 6, 6, 6);
    mat_mul(I_minus_KH_P, I_minus_KH_T, I_minus_KH_P_I_minus_KH_T, 6, 6, 6);

    // Compute K*R*K'
    mat_transpose(K, K_T, 6, 3);
    mat_mul(K, R, KR, 6, 3, 3);
    mat_mul(KR, K_T, KRK_T, 6, 3, 6);

    // P_new = (I - K*H)*P*(I - K*H)' + K*R*K'
    mat_add(I_minus_KH_P_I_minus_KH_T, KRK_T, P_new, 6, 6);

    // Write updated covariance back to slate
    for (int i = 0; i < 36; i++)
    {
        slate->P_attitude[i] = P_new[i];
    }

    // Update log frobenius norm
    slate->P_attitude_log_frobenius = mat_log_frobenius(slate->P_attitude, 6);

    // Check for NaN in state or covariance and reinitialize if detected
    if (std::isnan(slate->p_eci_to_body[0]) ||
        std::isnan(slate->p_eci_to_body[1]) ||
        std::isnan(slate->p_eci_to_body[2]) ||
        std::isnan(slate->q_eci_to_body[0]) ||
        std::isnan(slate->q_eci_to_body[1]) ||
        std::isnan(slate->q_eci_to_body[2]) ||
        std::isnan(slate->q_eci_to_body[3]) ||
        std::isnan(slate->b_gyro_drift[0]) ||
        std::isnan(slate->b_gyro_drift[1]) ||
        std::isnan(slate->b_gyro_drift[2]) ||
        std::isnan(slate->P_attitude_log_frobenius))
    {
        LOG_ERROR("[ekf] NaN detected in update (sensor '%c')! Reinitializing "
                  "filter...",
                  sensor_type);
        attitude_filter_init(slate);
        return;
    }

    // LOG_DEBUG("[ekf] innovation = [%.6f, %.6f, %.6f]", innovation[0],
    //           innovation[1], innovation[2]);
    // LOG_DEBUG("[ekf] q_eci_to_body = [%.6f, %.6f, %.6f, %.6f]",
    //           slate->q_eci_to_body[0], slate->q_eci_to_body[1],
    //           slate->q_eci_to_body[2], slate->q_eci_to_body[3]);
    // LOG_DEBUG("[ekf] b_gyro_drift = [%.6f, %.6f, %.6f]", slate->b_gyro_drift[0],
    //           slate->b_gyro_drift[1], slate->b_gyro_drift[2]);
    // LOG_DEBUG("[ekf] P_attitude_log_frobenius = %.6f",
    //           slate->P_attitude_log_frobenius);
}

#ifdef TEST
#include "tools/attitude_propagator.h"
#include "tools/orbit_prop.h"
#include <random>
// random float from standard normal distribution N(0,1)
float random_gaussian() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> dis(0.0f, 1.0f);
    return dis(gen);
}
// random float3 from standard normal distribution
float3 random_gaussian_float3() {
    return float3(random_gaussian(), random_gaussian(), random_gaussian());
}
void ekf_propagator_test(slate_t *slate) {
    // test with propagate_attitude function from tools
    printf("\n><=><=><=><=><= EKF Test via Attitude Propagator! ><=><=><=><=><=\n");
    // define ground truth state vector
    float4 q_eci2body = {0.0f, 0.0f, 0.0f, 1.0f};
    float3 w_body = {0.01f, 0.02f, 0.03f};
    float3 r_eci;
    float3 v_eci;
    float3 b_eci = {1.0f, 0.0f, 0.0f};
    float3 mu_mt = {0.0f, 0.0f, 0.0f};
    float3 tau_rw = {0.0f, 0.0f, 0.0f};
    float3 b_gyro_drift = {0.0f, 0.0f, 0.0f};
    float t = 0.0f;

    // initialize attitude filter
    attitude_filter_init(slate);

    // reference vectors in inertial frame
    slate->sun_vector_eci = {0.0f, 1.0f, 0.0f};
    slate->b_eci = b_eci;

    // propagate attitude, orbit, update filter
    float tf = 900.0f;
    float dt = 0.05f;
    int steps = static_cast<int>(tf / dt);
    for (int i = 0; i < steps; i++) {
        propagate_polar_orbit(r_eci, v_eci, t);
        propagate_attitude(q_eci2body, w_body, r_eci, v_eci, b_eci, mu_mt, tau_rw, dt);

        // Simulate gyro measurement (ground truth + drift)
        b_gyro_drift = b_gyro_drift + sqrtf(IMU_DRIFT_VARIANCE * dt) * random_gaussian_float3(); // Gaussian random walk
        slate->w_body = w_body + b_gyro_drift;

        // Manually set timing for controlled dt in test environment
        absolute_time_t now = get_absolute_time();
        slate->af_last_propagate_time = now - static_cast<uint64_t>(dt * 1e6);

        // Propagate EKF using gyro measurement
        attitude_filter_propagate(slate);

        // After first propagate (which skips if af_last_propagate_time was 0),
        // ensure timing is set correctly for subsequent iterations
        if (i == 0) {
            slate->af_last_propagate_time = now;
        }

        // Simulate vector measurements by rotating ECI references to body frame
        slate->sun_vector_body = qrot(q_eci2body, slate->sun_vector_eci);
        slate->b_body = qrot(q_eci2body, slate->b_eci);

        // DEBUG: Compare ground truth rotation to EKF estimate
        if (i == 100) {
            float3 sun_predicted = mul(mrp_to_dcm(slate->p_eci_to_body), slate->sun_vector_eci);
            float3 sun_actual = slate->sun_vector_body;
            LOG_INFO("[DEBUG] Sun actual=[%.3f,%.3f,%.3f], predicted=[%.3f,%.3f,%.3f]",
                     sun_actual.x, sun_actual.y, sun_actual.z,
                     sun_predicted.x, sun_predicted.y, sun_predicted.z);
            float3 mrp_true = quat_to_mrp(q_eci2body);
            LOG_INFO("[DEBUG] MRP_true=[%.3f,%.3f,%.3f], MRP_est=[%.3f,%.3f,%.3f]",
                     mrp_true.x, mrp_true.y, mrp_true.z,
                     slate->p_eci_to_body.x, slate->p_eci_to_body.y, slate->p_eci_to_body.z);
            LOG_INFO("[DEBUG] q_true=[%.3f,%.3f,%.3f,%.3f], q_est=[%.3f,%.3f,%.3f,%.3f]",
                     q_eci2body[0], q_eci2body[1], q_eci2body[2], q_eci2body[3],
                     slate->q_eci_to_body[0], slate->q_eci_to_body[1],
                     slate->q_eci_to_body[2], slate->q_eci_to_body[3]);
        }

        if (i % 1000 == 0) {
            // log state health metrics every 1000 steps
            float mrp_norm = length(slate->p_eci_to_body);
            printf("[ekf] MRP norm = %.6f, P_log_frob = %.6f\n", mrp_norm, slate->P_attitude_log_frobenius);
        }

        // Update EKF with vector measurements
        attitude_filter_update(slate, 'S');
        attitude_filter_update(slate, 'M');
        t = t + dt;
    }
    printf("q_eci2body (ground truth): [%.6f, %.6f, %.6f, %.6f]\n", q_eci2body[0], q_eci2body[1], q_eci2body[2], q_eci2body[3]);
    printf("q_eci2body (estimated): [%.6f, %.6f, %.6f, %.6f]\n", slate->q_eci_to_body[0], slate->q_eci_to_body[1], slate->q_eci_to_body[2], slate->q_eci_to_body[3]);
    printf("w_body (ground truth): [%.6f, %.6f, %.6f]\n", w_body[0], w_body[1], w_body[2]);
    printf("w_body (estimated): [%.6f, %.6f, %.6f]\n", slate->w_body[0], slate->w_body[1], slate->w_body[2]);
    printf("r_eci (ground truth): [%.6f, %.6f, %.6f]\n", r_eci[0], r_eci[1], r_eci[2]);
    printf("v_eci (ground truth): [%.6f, %.6f, %.6f]\n", v_eci[0], v_eci[1], v_eci[2]);
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_time_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Benchmarking EKF! ><=><=><=><=><=\n");
    attitude_filter_init(slate);
    int time = 3600;  // seconds
    float dt = 0.01f; // 10ms propagate rate
    int steps = 50000;

    LOG_INFO("Propagating EKF for %d steps...", steps);
    slate->w_body = {0.1f, 0.2f, 0.3f};
    for (int i = 0; i < steps; i++)
    {
        // Simulate dt by setting last propagate time
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);
    }
    LOG_INFO("Finished propagating EKF for %d steps", steps);

    attitude_filter_init(slate);
    LOG_INFO("Updating EKF for %d steps...", steps);
    slate->w_body = {0.0f, 0.0f, 0.0f};
    slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
    slate->sun_vector_body = {0.0f, -1.0f, 0.0f};
    for (int i = 0; i < steps; i++)
    {
        attitude_filter_update(slate, 'S'); // Update with sun vector
        attitude_filter_update(slate, 'M'); // Update with magnetic field vector
    }
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_convergence_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Testing EKF Convergence! ><=><=><=><=><=\n");
    attitude_filter_init(slate);
    int time = 3600;  // seconds
    float dt = 0.01f; // 10ms propagate rate
    int steps = time / dt;
    int sun_measurement_hz = 20; // 20 Hz sun sensor
    int b_measurement_hz = 10;   // 10 Hz magnetometer
    int sun_measurement_interval =
        static_cast<int>(1.0f / (sun_measurement_hz * dt));
    int b_measurement_interval =
        static_cast<int>(1.0f / (b_measurement_hz * dt));

    slate->w_body = {0.0f, 0.0f, 0.0f};
    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        // Simulate dt by setting last propagate time
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);
        // Vectors in inertial frame
        slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
        slate->b_eci = {0.0f, 1.0f, 0.0f};

        // Vectors in body frame
        slate->sun_vector_body = {0.0f, -1.0f, 0.0f};
        slate->b_body = {1.0f, 0.0f, 0.0f};

        if (i % sun_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'S'); // Update with sun vector
        }
        if (i % b_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'M'); // Update with magnetic field
                                                // vector
        }
    }
    quaternion q_expected = {0.0f, 0.0f, 0.7071068f, 0.7071068f}; // 90 deg rot
    float3 p_expected = quat_to_mrp(q_expected);
    if (length(slate->p_eci_to_body - p_expected) > 0.01f)
    {
        LOG_ERROR("EKF did not converge to expected MRP!");
        passed = false;
    }
    if (length(slate->q_eci_to_body - q_expected) > 0.01f)
    {
        LOG_ERROR("EKF did not converge to expected quaternion!");
        passed = false;
    }
    LOG_INFO("Final quaternion [x,y,z,w]: %.6f, %.6f, %.6f, %.6f",
             slate->q_eci_to_body[0], slate->q_eci_to_body[1],
             slate->q_eci_to_body[2], slate->q_eci_to_body[3]);
    LOG_INFO("Final MRP [x,y,z]: %.6f, %.6f, %.6f", slate->p_eci_to_body[0],
             slate->p_eci_to_body[1], slate->p_eci_to_body[2]);
    LOG_INFO("Final gyro bias [x,y,z]: %.6f, %.6f, %.6f",
             slate->b_gyro_drift[0], slate->b_gyro_drift[1],
             slate->b_gyro_drift[2]);
    LOG_INFO("Final P log frobenius: %.6f", slate->P_attitude_log_frobenius);
    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_mrp_wrapping_test(slate_t *slate)
{
    printf(
        "\n><=><=><=> Testing MRP Wrapping (No Singularity)! ><=><=><=><=\n");
    attitude_filter_init(slate);
    int steps = 2000;
    float dt = 0.01f;
    slate->w_body = {5.0f, 3.0f, 2.0f}; // High angular velocity

    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);
        float mrp_norm = length(slate->p_eci_to_body);

        if (mrp_norm > 1.0f)
        {
            LOG_ERROR("MRP norm exceeded 1.0! Norm: %.6f at step %d", mrp_norm,
                      i);
            passed = false;
            break;
        }

        if (std::isnan(slate->p_eci_to_body[0]))
        {
            LOG_ERROR("MRP contains NaN at step %d", i);
            passed = false;
            break;
        }
    }

    LOG_INFO("Final MRP norm: %.6f", length(slate->p_eci_to_body));
    LOG_INFO("Final quaternion norm: %.6f", length(slate->q_eci_to_body));
    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_high_rate_tumble_test(slate_t *slate)
{
    printf("\n><=><=><=> Testing High Rate Tumble! ><=><=><=><=\n");
    attitude_filter_init(slate);
    int steps = 500;
    float dt = 0.05f;
    slate->w_body = {10.0f, -8.0f, 6.0f}; // Fast tumble

    bool passed = true;
    float initial_w_mag = length(slate->w_body);

    for (int i = 0; i < steps; i++)
    {
        slate->w_body = slate->w_body * 0.995f; // Damp like bdot
        // Simulate dt by setting last propagate time
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);

        float mrp_norm = length(slate->p_eci_to_body);
        if (mrp_norm > 1.0f)
        {
            LOG_ERROR("MRP norm exceeded 1.0! Norm: %.6f at step %d", mrp_norm,
                      i);
            passed = false;
            break;
        }

        if (std::isnan(slate->p_eci_to_body[0]) ||
            std::isnan(slate->q_eci_to_body[0]))
        {
            LOG_ERROR("State contains NaN at step %d", i);
            passed = false;
            break;
        }
    }

    float final_w_mag = length(slate->w_body);
    LOG_INFO("Initial w magnitude: %.6f rad/s", initial_w_mag);
    LOG_INFO("Final w magnitude: %.6f rad/s", final_w_mag);
    LOG_INFO("Final MRP norm: %.6f", length(slate->p_eci_to_body));
    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_convergence_logging_test(slate_t *slate)
{
    printf(
        "\n><=><=><=> EKF Convergence Logging Test (1M steps) ><=><=><=><=\n");
    printf("Logging format: "
           "step,time,P_attitude_log_frobenius,mrp_error,quat_error,p_x,p_y,p_"
           "z,q_w,q_x,"
           "q_y,q_z,b_x,b_y,b_z\n");
    printf("DATA_START\n");

    attitude_filter_init(slate);
    int steps = 10000000;
    float dt = 0.05f;            // 50ms propagate rate
    int sun_measurement_hz = 20; // 20 Hz sun sensor
    int b_measurement_hz = 10;   // 10 Hz magnetometer
    int sun_measurement_interval =
        static_cast<int>(1.0f / (sun_measurement_hz * dt));
    int b_measurement_interval =
        static_cast<int>(1.0f / (b_measurement_hz * dt));

    // Expected final state
    quaternion q_expected = {0.0f, 0.0f, 0.7071068f, 0.7071068f}; // 90 deg rot
    float3 p_expected = quat_to_mrp(q_expected);

    slate->w_body = {0.0f, 0.0f, 0.0f};

    // Log every N steps to reduce output size
    int log_interval = 1000;

    for (int i = 0; i < steps; i++)
    {
        // Simulate dt by setting last propagate time
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);

        // Vectors in inertial frame
        slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
        slate->b_eci = {0.0f, 1.0f, 0.0f};

        // Vectors in body frame (90 degree rotation around z-axis)
        slate->sun_vector_body = {0.0f, -1.0f, 0.0f};
        slate->b_body = {1.0f, 0.0f, 0.0f};

        if (i % sun_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'S'); // Update with sun vector
        }
        if (i % b_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'M'); // Update with magnetic field
        }

        // Log data at specified interval
        if (i % log_interval == 0)
        {
            float time_sec = i * dt;
            float mrp_error = length(slate->p_eci_to_body - p_expected);
            float quat_error = length(slate->q_eci_to_body - q_expected);

            printf("%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%"
                   ".6f,%.6f,%.6f\n",
                   i, time_sec, slate->P_attitude_log_frobenius, mrp_error,
                   quat_error, slate->p_eci_to_body.x, slate->p_eci_to_body.y,
                   slate->p_eci_to_body.z, slate->q_eci_to_body.w,
                   slate->q_eci_to_body.x, slate->q_eci_to_body.y,
                   slate->q_eci_to_body.z, slate->b_gyro_drift.x,
                   slate->b_gyro_drift.y, slate->b_gyro_drift.z);
        }
    }

    printf("DATA_END\n");
    printf("\nFinal State Summary:\n");
    LOG_INFO("Final quaternion [w,x,y,z]: %.6f, %.6f, %.6f, %.6f",
             slate->q_eci_to_body.w, slate->q_eci_to_body.x,
             slate->q_eci_to_body.y, slate->q_eci_to_body.z);
    LOG_INFO("Final MRP [x,y,z]: %.6f, %.6f, %.6f", slate->p_eci_to_body.x,
             slate->p_eci_to_body.y, slate->p_eci_to_body.z);
    LOG_INFO("Final gyro bias [x,y,z]: %.6f, %.6f, %.6f", slate->b_gyro_drift.x,
             slate->b_gyro_drift.y, slate->b_gyro_drift.z);
    LOG_INFO("Final P log frobenius: %.6f", slate->P_attitude_log_frobenius);
    LOG_INFO("MRP error: %.6f", length(slate->p_eci_to_body - p_expected));
    LOG_INFO("Quaternion error: %.6f",
             length(slate->q_eci_to_body - q_expected));
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_stationary_bias_test(slate_t *slate)
{
    printf("\n><=><=><=> Testing EKF Stationary with Bias! ><=><=><=><=\n");
    printf(
        "This test checks if the filter can estimate bias when NOT rotating\n");

    // Known true values
    float3 true_bias = {0.01f, -0.02f, 0.015f}; // rad/s - realistic gyro bias
    float3 true_angular_velocity = {0.0f, 0.0f,
                                    0.0f}; // NO rotation - stationary

    printf("True bias [x,y,z]: %.6f, %.6f, %.6f rad/s\n", true_bias.x,
           true_bias.y, true_bias.z);
    printf("True angular velocity: ZERO (stationary test)\n");

    attitude_filter_init(slate);

    // Test parameters
    int steps = 10000;
    float dt = 0.01f;
    int sun_measurement_hz = 20;
    int b_measurement_hz = 10;
    int sun_measurement_interval =
        static_cast<int>(1.0f / (sun_measurement_hz * dt));
    int b_measurement_interval =
        static_cast<int>(1.0f / (b_measurement_hz * dt));

    // True attitude stays at identity since no rotation
    quaternion q_true = {0.0f, 0.0f, 0.0f,
                         1.0f}; // Identity: (x,y,z,w)=(0,0,0,1)

    bool passed = true;

    for (int i = 0; i < steps; i++)
    {
        // Simulate gyro measurement: measured = true_velocity + bias = 0 + bias
        // = bias
        slate->w_body = true_angular_velocity + true_bias;

        // Propagate the filter
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);

        // Generate synthetic measurements (identity rotation)
        float3 sun_eci = {1.0f, 0.0f, 0.0f};
        float3 mag_eci = {0.0f, 1.0f, 0.0f};

        // Body frame = inertial frame (no rotation)
        slate->sun_vector_body = sun_eci;
        slate->b_body = mag_eci;
        slate->sun_vector_eci = sun_eci;
        slate->b_eci = mag_eci;

        // Update measurements
        if (i % sun_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'S');
        }
        if (i % b_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'M');
        }

        // Check for instability
        if (std::isnan(slate->p_eci_to_body[0]) ||
            length(slate->p_eci_to_body) > 1.0f)
        {
            LOG_ERROR("Filter became unstable at step %d!", i);
            passed = false;
            break;
        }

        // Log progress
        if (i % 1000 == 0)
        {
            float bias_error = length(slate->b_gyro_drift - true_bias);
            float quat_error = length(slate->q_eci_to_body - q_true);
            float mrp_norm = length(slate->p_eci_to_body);
            // Check attitude-bias coupling in covariance (P[3] = cov(p_x, b_x),
            // P[9] = cov(p_y, b_x), etc.)
            float max_coupling = fmaxf(
                fmaxf(fabsf(slate->P_attitude[3]), fabsf(slate->P_attitude[9])),
                fabsf(slate->P_attitude[15]));
            printf("Step %d: bias_error=%.6f, quat_error=%.6f, mrp_norm=%.6f, "
                   "P_diag=[%.2e,%.2e,%.2e,%.2e,%.2e,%.2e], coupling=%.2e\n",
                   i, bias_error, quat_error, mrp_norm, slate->P_attitude[0],
                   slate->P_attitude[7], slate->P_attitude[14],
                   slate->P_attitude[21], slate->P_attitude[28],
                   slate->P_attitude[35], max_coupling);
        }
    }

    printf("\n=== Final Results ===\n");
    float3 bias_error = slate->b_gyro_drift - true_bias;
    float bias_error_mag = length(bias_error);
    float quat_error_mag = length(slate->q_eci_to_body - q_true);

    printf("\nTrue bias:      [%.6f, %.6f, %.6f] rad/s\n", true_bias.x,
           true_bias.y, true_bias.z);
    printf("Estimated bias: [%.6f, %.6f, %.6f] rad/s\n", slate->b_gyro_drift.x,
           slate->b_gyro_drift.y, slate->b_gyro_drift.z);
    printf("Bias error magnitude: %.6f rad/s (%.3f deg/s)\n", bias_error_mag,
           bias_error_mag * RAD_TO_DEG);
    printf("Quaternion error magnitude: %.6f\n", quat_error_mag);

    float bias_tolerance = 0.005f;
    float quat_tolerance = 0.01f;

    if (bias_error_mag > bias_tolerance)
    {
        LOG_ERROR("Bias error too large! %.6f > %.6f rad/s", bias_error_mag,
                  bias_tolerance);
        passed = false;
    }

    if (quat_error_mag > quat_tolerance)
    {
        LOG_ERROR("Quaternion error too large! %.6f > %.6f", quat_error_mag,
                  quat_tolerance);
        passed = false;
    }

    printf("\n");
    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void ekf_rotation_with_bias_test(slate_t *slate)
{
    printf("\n><=><=><=> Testing EKF Rotation with Known Bias! ><=><=><=><=\n");

    // Known true values
    float3 true_bias = {0.01f, -0.02f, 0.015f}; // rad/s - realistic gyro bias
    float3 true_angular_velocity = {0.5f, 0.3f,
                                    -0.4f}; // rad/s - moderate rotation rate

    printf("True bias [x,y,z]: %.6f, %.6f, %.6f rad/s\n", true_bias.x,
           true_bias.y, true_bias.z);
    printf("True angular velocity [x,y,z]: %.6f, %.6f, %.6f rad/s\n",
           true_angular_velocity.x, true_angular_velocity.y,
           true_angular_velocity.z);

    attitude_filter_init(slate);

    // Test parameters
    int steps = 10000;
    float dt = 0.01f;            // 10ms timestep, 100Hz propagation
    int sun_measurement_hz = 20; // 20 Hz sun sensor
    int b_measurement_hz = 10;   // 10 Hz magnetometer
    int sun_measurement_interval =
        static_cast<int>(1.0f / (sun_measurement_hz * dt));
    int b_measurement_interval =
        static_cast<int>(1.0f / (b_measurement_hz * dt));

    // Track expected attitude by integrating true angular velocity
    quaternion q_true = {0.0f, 0.0f, 0.0f,
                         1.0f}; // Identity: (x,y,z,w)=(0,0,0,1)

    bool passed = true;

    for (int i = 0; i < steps; i++)
    {
        // Simulate gyro measurement: measured = true_velocity + bias
        slate->w_body = true_angular_velocity + true_bias;

        // Propagate the filter
        slate->af_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);
        attitude_filter_propagate(slate);

        // Integrate true attitude (ground truth) using the TRUE angular
        // velocity q_dot = 0.5 * q * [0, w_true]
        quaternion w_quat = {0.0f, true_angular_velocity.x,
                             true_angular_velocity.y, true_angular_velocity.z};
        quaternion q_dot = 0.5f * qmul(q_true, w_quat);
        q_true = q_true + dt * q_dot;
        q_true = normalize(q_true);

        // Generate synthetic measurements in body frame
        // For simplicity, use fixed inertial vectors
        float3 sun_eci = {1.0f, 0.0f, 0.0f};
        float3 mag_eci = {0.0f, 1.0f, 0.0f};

        // Rotate to body frame using TRUE attitude
        float3x3 dcm_true = quaternion_to_dcm(q_true);
        slate->sun_vector_body = mul(dcm_true, sun_eci);
        slate->b_body = mul(dcm_true, mag_eci);

        // Inertial frame vectors (what the filter expects)
        slate->sun_vector_eci = sun_eci;
        slate->b_eci = mag_eci;

        // Update measurements at their respective rates
        if (i % sun_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'S');
        }
        if (i % b_measurement_interval == 0)
        {
            attitude_filter_update(slate, 'M');
        }

        // Check for instability
        if (std::isnan(slate->p_eci_to_body[0]) ||
            length(slate->p_eci_to_body) > 1.0f)
        {
            LOG_ERROR("Filter became unstable at step %d!", i);
            passed = false;
            break;
        }

        // Log progress periodically
        if (i % 1000 == 0)
        {
            float bias_error = length(slate->b_gyro_drift - true_bias);
            float quat_error = length(slate->q_eci_to_body - q_true);
            printf("Step %d: bias_error=%.6f rad/s, quat_error=%.6f\n", i,
                   bias_error, quat_error);
        }
    }

    // Final analysis
    printf("\n=== Final Results ===\n");

    float3 bias_error = slate->b_gyro_drift - true_bias;
    float bias_error_mag = length(bias_error);
    float quat_error_mag = length(slate->q_eci_to_body - q_true);

    printf("\nTrue bias:      [%.6f, %.6f, %.6f] rad/s\n", true_bias.x,
           true_bias.y, true_bias.z);
    printf("Estimated bias: [%.6f, %.6f, %.6f] rad/s\n", slate->b_gyro_drift.x,
           slate->b_gyro_drift.y, slate->b_gyro_drift.z);
    printf("Bias error:     [%.6f, %.6f, %.6f] rad/s\n", bias_error.x,
           bias_error.y, bias_error.z);
    printf("Bias error magnitude: %.6f rad/s (%.3f deg/s)\n", bias_error_mag,
           bias_error_mag * RAD_TO_DEG);

    printf("\nTrue quaternion:      [%.6f, %.6f, %.6f, %.6f]\n", q_true.w,
           q_true.x, q_true.y, q_true.z);
    printf("Estimated quaternion: [%.6f, %.6f, %.6f, %.6f]\n",
           slate->q_eci_to_body.w, slate->q_eci_to_body.x,
           slate->q_eci_to_body.y, slate->q_eci_to_body.z);
    printf("Quaternion error magnitude: %.6f\n", quat_error_mag);

    printf("\nEstimated MRP: [%.6f, %.6f, %.6f]\n", slate->p_eci_to_body.x,
           slate->p_eci_to_body.y, slate->p_eci_to_body.z);
    printf("P log frobenius: %.6f\n", slate->P_attitude_log_frobenius);
    printf("Attitude uncertainty (1-sigma): [%.3f, %.3f, %.3f] deg\n",
           sqrtf(slate->P_attitude[0]) * RAD_TO_DEG,
           sqrtf(slate->P_attitude[7]) * RAD_TO_DEG,
           sqrtf(slate->P_attitude[14]) * RAD_TO_DEG);

    // Pass/fail criteria
    float bias_tolerance = 0.005f; // 0.005 rad/s = 0.286 deg/s
    float quat_tolerance = 0.05f;  // Quaternion distance

    if (bias_error_mag > bias_tolerance)
    {
        LOG_ERROR("Bias error too large! %.6f > %.6f rad/s", bias_error_mag,
                  bias_tolerance);
        passed = false;
    }

    if (quat_error_mag > quat_tolerance)
    {
        LOG_ERROR("Quaternion error too large! %.6f > %.6f", quat_error_mag,
                  quat_tolerance);
        passed = false;
    }

    printf("\n");
    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}
#endif