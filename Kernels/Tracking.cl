/**
 * @brief Mixes each complex sample with a numerically controlled oscillator (NCO) replica
 * given a per-sample carrier phase, rotating the input in place.
 *
 * @param g_data Complex input/output data (interleaved Re, Im), rotated in place.
 * @param phasePoints Per-sample carrier phase (radians).
 * @param l_data Unused local memory buffer, kept for work-group dispatch symmetry.
 * @param points_per_group Points processed per work-group.
 */
__kernel void ncoMultiplicate(__global float2 *g_data,
                              __global float *phasePoints,
                              __local float2 *l_data, uint points_per_group) {
  uint points_per_item = points_per_group / get_local_size(0);
  uint l_addr = get_local_id(0) * points_per_item;
  uint g_addr = get_group_id(0) * points_per_group + l_addr;

  for (uint i = 0; i < points_per_item; i += 4) {
    uint4 index = (uint4)(g_addr, g_addr + 1, g_addr + 2, g_addr + 3);

    float2 x1 = g_data[index.s0];
    float2 x2 = g_data[index.s1];
    float2 x3 = g_data[index.s2];
    float2 x4 = g_data[index.s3];

    float p1 = phasePoints[index.s0];
    float p2 = phasePoints[index.s1];
    float p3 = phasePoints[index.s2];
    float p4 = phasePoints[index.s3];

    float c1 = cos(p1), s1 = sin(p1);
    float c2 = cos(p2), s2 = sin(p2);
    float c3 = cos(p3), s3 = sin(p3);
    float c4 = cos(p4), s4 = sin(p4);

    g_data[index.s0] = (float2)(x1.s0 * c1 - x1.s1 * s1, x1.s0 * s1 + x1.s1 * c1);
    g_data[index.s1] = (float2)(x2.s0 * c2 - x2.s1 * s2, x2.s0 * s2 + x2.s1 * c2);
    g_data[index.s2] = (float2)(x3.s0 * c3 - x3.s1 * s3, x3.s0 * s3 + x3.s1 * c3);
    g_data[index.s3] = (float2)(x4.s0 * c4 - x4.s1 * s4, x4.s0 * s4 + x4.s1 * c4);

    l_addr += 4;
    g_addr += 4;
  }
}