/**
 * @author Niklas Vainio
 * @date 2025-05-03
 *
 * This file contains utils for converting sun-magnetic vector pairs into
 * rotation matrices and quaternions
 */

#include "traid_solver.h"

/**
 * @brief Compute triad frame matrix from sun and magnetic field vectors
 *
 * @param s
 * @param b
 * @return float3x3
 */
float3x3 compute_triad_matrix(float3 s, float3 b)
{
    const float3 s_hat = normalize(s);
    const float3 b_hat = normalize(b);

    const float3 v1 = s_hat;
    const float3 v2 = normalize(cross(s_hat, b_hat));
    const float3 v3 = cross(v1, v2);

    // (this initializer is column major)
    return float3x3{v1.x, v1.y, v1.z, v2.x, v2.y, v2.x, v3.x, v3.y, v3.z};
}

/**
 * @brief Compute the eci to local quaternion given local frame and global frame
 * sun-magnetic measurements
 *
 * @param s_eci
 * @param b_eci
 * @param s_local
 * @param b_local
 * @return quaternion
 */
quaternion compute_attitude_quaternion(float3 s_eci, float3 b_eci,
                                       float3 s_local, float3 b_local)
{
    float3x3 R_local = compute_triad_matrix(s_local, s_eci);
    float3x3 R_eci = compute_triad_matrix(s_eci, b_eci);

    float3x3 R_eci_to_local = R_local * transpose(R_eci);

    quaternion q_eci_to_local = rotation_quat()
}