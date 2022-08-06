#define mask_left fft_index
#define mask_right stage
#define shift_pos N2
#define angle size
#define start br.s0
#define cosine x3.s0
#define sine x3.s1
#define wk x2

/**
 * @brief This method functionality defined below
 * - Loads data from global memory, computes 4-point FFT, and stores the result
 * in local memory.
 * - To ease the computation, before computation, the input data stored in
 * bit-reversal structure.
 * - It computes bigger FFT sizes in a loop till it reaches the full length of
 * given input data
 *
 * - Source :
 *   Scarpino, M. (2012). OpenCL in action: How to accelerate graphics and
 * computation. Manning. CH 14. Signal processing and the fast Fourier transform
 *
 * @param g_data complex input of fft algorithm. It is sequenced as Re0, Im0,
 * Re1, Im1...
 * @param l_data complex output of computed fft sequence. Stored in local memory
 * @param points_per_group equals to points_per_group = local_mem_size /
 * (2*sizeof(float)). It is boundary for maximum computable fft length
 * @param size length of the FFT
 * @param dir 1 for FFT, -1 for IFFT
 */
__kernel void fft_init(__global float2 *g_data, __local float2 *l_data,
                       uint points_per_group, uint size, int dir) {

  uint4 br, index;
  uint points_per_item, g_addr, l_addr, i, fft_index, stage, N2;
  float2 x1, x2, x3, x4, sum12, diff12, sum34, diff34;

  points_per_item = points_per_group / get_local_size(0);
  l_addr = get_local_id(0) * points_per_item;
  g_addr = get_group_id(0) * points_per_group + l_addr;

  /* Load data from bit-reversed addresses and perform 4-point FFTs */
  for (i = 0; i < points_per_item; i += 4) {
    index = (uint4)(g_addr, g_addr + 1, g_addr + 2, g_addr + 3);
    mask_left = size / 2;
    mask_right = 1;
    shift_pos = log2((float)size) - 1;
    br = (index << shift_pos) & mask_left;
    br |= (index >> shift_pos) & mask_right;

    /* Bit-reverse addresses */
    while (shift_pos > 1) {
      shift_pos -= 2;
      mask_left >>= 1;
      mask_right <<= 1;
      br |= (index << shift_pos) & mask_left;
      br |= (index >> shift_pos) & mask_right;
    }

    /* Load global data */
    x1 = g_data[br.s0];
    x2 = g_data[br.s1];
    x3 = g_data[br.s2];
    x4 = g_data[br.s3];

    /* Perform 4 element FFT */
    sum12 = x1 + x2;
    diff12 = x1 - x2;
    sum34 = x3 + x4;
    diff34 = (float2)(x3.s1 - x4.s1, x4.s0 - x3.s0) * dir;
    l_data[l_addr] = sum12 + sum34;
    l_data[l_addr + 1] = diff12 + diff34;
    l_data[l_addr + 2] = sum12 - sum34;
    l_data[l_addr + 3] = diff12 - diff34;
    l_addr += 4;
    g_addr += 4;
  }

  /* Perform initial stages of the FFT - each of length N2*2 */
  for (N2 = 4; N2 < points_per_item; N2 <<= 1) {
    l_addr = get_local_id(0) * points_per_item;
    for (fft_index = 0; fft_index < points_per_item; fft_index += 2 * N2) {
      x1 = l_data[l_addr];
      l_data[l_addr] += l_data[l_addr + N2];
      l_data[l_addr + N2] = x1 - l_data[l_addr + N2];
      for (i = 1; i < N2; i++) {
        /* Compute Trigonometric Terms */
        cosine = cos(M_PI_F * i / N2);
        sine = dir * sin(M_PI_F * i / N2);
        wk = (float2)(l_data[l_addr + N2 + i].s0 * cosine +
                          l_data[l_addr + N2 + i].s1 * sine,
                      l_data[l_addr + N2 + i].s1 * cosine -
                          l_data[l_addr + N2 + i].s0 * sine);
        /* Compute Frequency Components */
        l_data[l_addr + N2 + i] = l_data[l_addr + i] - wk;
        l_data[l_addr + i] += wk;
      }
      l_addr += 2 * N2;
    }
  }
  barrier(CLK_LOCAL_MEM_FENCE);

  /* Perform FFT with other items in group - each of length N2*2 */
  stage = 2;
  for (N2 = points_per_item; N2 < points_per_group; N2 <<= 1) {
    start = (get_local_id(0) + (get_local_id(0) / stage) * stage) *
            (points_per_item / 2);
    angle = start % (N2 * 2);
    for (i = start; i < start + points_per_item / 2; i++) {
      cosine = cos(M_PI_F * angle / N2);
      sine = dir * sin(M_PI_F * angle / N2);
      wk = (float2)(l_data[N2 + i].s0 * cosine + l_data[N2 + i].s1 * sine,
                    l_data[N2 + i].s1 * cosine - l_data[N2 + i].s0 * sine);
      l_data[N2 + i] = l_data[i] - wk;
      l_data[i] += wk;
      angle++;
    }
    stage <<= 1;
    barrier(CLK_LOCAL_MEM_FENCE);
  }

  /* Store results in global memory */
  l_addr = get_local_id(0) * points_per_item;
  g_addr = get_group_id(0) * points_per_group + l_addr;
  for (i = 0; i < points_per_item; i += 4) {
    g_data[g_addr] = l_data[l_addr];
    g_data[g_addr + 1] = l_data[l_addr + 1];
    g_data[g_addr + 2] = l_data[l_addr + 2];
    g_data[g_addr + 3] = l_data[l_addr + 3];
    g_addr += 4;
    l_addr += 4;
  }
}

/**
 * @brief This method functionality defined below
 * - Computes the final merged FFT computation after fft_init kernel is called.
 * - Bit reversal structure is converted to sequential structure.
 *
 * - Source :
 *   Scarpino, M. (2012). OpenCL in action: How to accelerate graphics and
 * computation. Manning. CH 14. Signal processing and the fast Fourier transform
 *
 * @param g_data complex input of fft algorithm. Output is written to this field
 * as well.
 * @param stage fft stage input. It should be iterated on the host side for each
 * stage.
 * @param points_per_group equals to points_per_group = local_mem_size /
 * (2*sizeof(float)). It is boundary for maximum computable fft length
 * @param dir 1 for FFT, -1 for IFFT
 */
__kernel void fft_stage(__global float2 *g_data, uint stage,
                        uint points_per_group, int dir) {

  uint points_per_item, addr, N, ang, i;
  float c, s;
  float2 input1, input2, w;
  /* Assign work item position */
  points_per_item = points_per_group / get_local_size(0);
  addr = (get_group_id(0) + (get_group_id(0) / stage) * stage) *
             (points_per_group / 2) +
         get_local_id(0) * (points_per_item / 2);
  N = points_per_group * (stage / 2);
  ang = addr % (N * 2);

  for (i = addr; i < addr + points_per_item / 2; i++) {
    c = cos(M_PI_F * ang / N);
    s = dir * sin(M_PI_F * ang / N);
    input1 = g_data[i];
    input2 = g_data[i + N];
    w = (float2)(input2.s0 * c + input2.s1 * s, input2.s1 * c - input2.s0 * s);
    g_data[i] = input1 + w;
    g_data[i + N] = input1 - w;
    ang++;
  }
}

/**
 * @brief This method is only called for IFFT operation. Each computed element
 * is divided to scale factor as only forward transformation has processing
 * gain.
 *
 * @param g_data complex input of fft algorithm. Output is written to this field
 * as well.
 * @param points_per_group equals to points_per_group = local_mem_size /
 * (2*sizeof(float)). It is boundary for maximum computable fft length
 * @param scale length of the fft computation. It denotes to 1/N multiplication
 * of Inverse Fourier transform.
 */
__kernel void fft_scale(__global float2 *g_data, uint points_per_group,
                        uint scale) {

  uint points_per_item, addr, i;

  points_per_item = points_per_group / get_local_size(0);
  addr = get_group_id(0) * points_per_group + get_local_id(0) * points_per_item;

  for (i = addr; i < addr + points_per_item; i++) {
    g_data[i] /= scale;
  }
}

/**
 */
__kernel void complexMultiplier(__global float2 *a, __global float2 *b,
                                __global float2 *c,
                                unsigned int points_per_group) {
  uint points_per_item, addr, i;

  points_per_item = points_per_group / get_local_size(0);
  addr = get_group_id(0) * points_per_group + get_local_id(0) * points_per_item;

  for (i = addr; i < addr + points_per_item; i++) {
    c[i].x = (a[i].x * b[i].x) - (a[i].y * b[i].y);
    c[i].y = (a[i].y * b[i].x) + (a[i].x * b[i].y);
  }
}

/**
 */
__kernel void absolute(__global float2 *a, __global float2 *c,
                       unsigned int points_per_group) {
  //|(a + ib)| = (a^2) + (b^2)
  uint points_per_item, addr, i;

  points_per_item = points_per_group / get_local_size(0);
  addr = get_group_id(0) * points_per_group + get_local_id(0) * points_per_item;

  for (i = addr; i < addr + points_per_item; i++) {
    c[i].x = (a[i].x * a[i].x) + (a[i].y * a[i].y);
  }
}

/**
 */
__kernel void sum(__global float *input, __global float *sumValue,
                  __local float *localMemory1) {

  const size_t globalId = get_global_id(0);
  const size_t localId = get_local_id(0);

  // Find sum of input
  localMemory1[localId] = input[globalId];

  barrier(CLK_LOCAL_MEM_FENCE);
  size_t blockSize = get_local_size(0);
  size_t halfBlockSize = blockSize / 2;
  while (halfBlockSize > 0) {
    if (localId < halfBlockSize) {
      localMemory1[localId] += localMemory1[localId + halfBlockSize];
      if ((halfBlockSize * 2) < blockSize) {
        if (localId == 0) {
          localMemory1[localId] += localMemory1[localId + (blockSize - 1)];
        }
      }
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    blockSize = halfBlockSize;
    halfBlockSize = blockSize / 2;
  }
  if (localId == 0) {
    sumValue[get_group_id(0)] = localMemory1[0];
  }
}
