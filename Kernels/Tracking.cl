__kernel void ncoMultiplicate(__global float2 *g_data,
                              __global float *phasePoints,
                              __local float2 *l_data, uint points_per_group) {
  uint points_per_item, g_addr, l_addr, i;
  uint4 index;
  float2 x1, x2, x3, x4;
  float p1, p2, p3, p4;
  // float freqScaled = freqeuncy;
  // float4 phasePoint;
  // float4 cosine;
  // float4 sine;

  points_per_item = points_per_group / get_local_size(0);
  l_addr = get_local_id(0) * points_per_item;
  g_addr = get_group_id(0) * points_per_group + l_addr;

  for (i = 0; i < points_per_item; i += 4) {
    index = (uint4)(g_addr, g_addr + 1, g_addr + 2, g_addr + 3);

    x1 = g_data[index.s0];
    x2 = g_data[index.s1];
    x3 = g_data[index.s2];
    x4 = g_data[index.s3];

    p1 = phasePoints[index.s0];
    p2 = phasePoints[index.s0];
    p3 = phasePoints[index.s0];
    p4 = phasePoints[index.s0];

    // phasePoint.s0 = (M_2_PI_F * sampleTime * freqScaled * l_addr) +
    // phaseOffset; phasePoint.s1 =
    //     (M_2_PI_F * sampleTime * freqScaled * (l_addr + 1)) + phaseOffset;
    // phasePoint.s2 =
    //     (M_2_PI_F * sampleTime * freqScaled * (l_addr + 2)) + phaseOffset;
    // phasePoint.s3 =
    //     (M_2_PI_F * sampleTime * freqScaled * (l_addr + 3)) + phaseOffset;

    // cosine = cos(phasePoint);
    // sine = sin(phasePoint);

    // l_data[l_addr].s0 = (x1.s0 * cosine.s0 - x1.s1 * sine.s0);
    // l_data[l_addr].s1 = (x1.s0 * sine.s0 + x1.s1 * cosine.s0);
    // l_data[l_addr + 1].s0 = (x2.s0 * cosine.s1 - x2.s1 * sine.s1);
    // l_data[l_addr + 1].s1 = (x2.s0 * sine.s1 + x2.s1 * cosine.s1);
    // l_data[l_addr + 2].s0 = (x3.s0 * cosine.s2 - x3.s1 * sine.s2);
    // l_data[l_addr + 2].s1 = (x3.s0 * sine.s2 + x3.s1 * cosine.s2);
    // l_data[l_addr + 3].s0 = (x4.s0 * cosine.s3 - x4.s1 * sine.s3);
    // l_data[l_addr + 3].s1 = (x4.s0 * sine.s3 + x4.s1 * cosine.s3);

    l_data[l_addr].s0 = p1;
    l_data[l_addr].s1 = 0.0f;
    l_data[l_addr + 1].s0 = p2;
    l_data[l_addr + 1].s1 = 0.0f;
    l_data[l_addr + 2].s0 = p3;
    l_data[l_addr + 2].s1 = 0.0f;
    l_data[l_addr + 3].s0 = p4;
    l_data[l_addr + 3].s1 = 0.0f;

    l_addr += 4;
    g_addr += 4;
    barrier(CLK_LOCAL_MEM_FENCE);
  }
  barrier(CLK_LOCAL_MEM_FENCE);
  g_data[get_global_id(0)] = l_data[get_global_id(0)];
}