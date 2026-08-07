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
 * @param g_data Complex input/output data (interleaved Re, Im).
 * @param l_data Local memory buffer.
 * @param points_per_group Maximum computable FFT points per workgroup.
 * @param size Length of the FFT.
 * @param dir 1 for FFT, -1 for IFFT.
 */
__kernel void fft_init(__global float2 *g_data, __local float2 *l_data,
                       uint points_per_group, uint size, int dir) {

  uint4 br, index;
  uint points_per_item, g_addr, l_addr, i;
  uint mask_left, mask_right, shift_pos, angle, start, N2, fft_index, stage;
  float2 x1, x2, x3, x4, sum12, diff12, sum34, diff34, wk;
  float cosine, sine;

  uint fft_base, local_addr;
  const uint shift_pos_init = (uint)log2((float)size) - 1;

  points_per_item = points_per_group / get_local_size(0);
  l_addr = get_local_id(0) * points_per_item;
  g_addr = get_group_id(0) * points_per_group + l_addr;

  /* Load data from bit-reversed addresses and perform 4-point FFTs.
   * Addresses are bit-reversed relative to the enclosing size-point FFT, so
   * when points_per_group == size each work-group transforms its own
   * independent FFT (batch mode); when points_per_group < size the groups
   * jointly transform one large FFT exactly as before. */
  for (i = 0; i < points_per_item; i += 4) {
    fft_base = (g_addr / size) * size;
    local_addr = g_addr - fft_base;
    index = (uint4)(local_addr, local_addr + 1, local_addr + 2, local_addr + 3);
    mask_left = size / 2;
    mask_right = 1;
    shift_pos = shift_pos_init;
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
    x1 = g_data[fft_base + br.s0];
    x2 = g_data[fft_base + br.s1];
    x3 = g_data[fft_base + br.s2];
    x4 = g_data[fft_base + br.s3];

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
        cosine = native_cos(M_PI_F * i / N2);
        sine = dir * native_sin(M_PI_F * i / N2);
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
      cosine = native_cos(M_PI_F * angle / N2);
      sine = dir * native_sin(M_PI_F * angle / N2);
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
  points_per_item = points_per_group / get_local_size(0);
  addr = (get_group_id(0) + (get_group_id(0) / stage) * stage) *
             (points_per_group / 2) +
         get_local_id(0) * (points_per_item / 2);
  N = points_per_group * (stage / 2);
  ang = addr % (N * 2);

  for (i = addr; i < addr + points_per_item / 2; i++) {
    c = native_cos(M_PI_F * ang / N);
    s = dir * native_sin(M_PI_F * ang / N);
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
 * @brief Element-wise complex multiplication with an optional circular offset
 * on the second operand: c[i] = a[i] * b[b_base + ((i + offset) mod length)].
 * Passing offset = 0 gives a plain element-wise product; a nonzero offset
 * implements a frequency-domain circular shift of b without any host-side
 * copy. b_base addresses one vector inside a pool buffer holding several
 * length-sized vectors back to back.
 *
 * @param a first complex input vector.
 * @param b second complex input vector, read with the circular offset.
 * @param c complex output vector.
 * @param points_per_group number of points processed per workgroup.
 * @param length element count of the vectors.
 * @param offset circular read offset into b, must be in [0, length).
 * @param b_base element offset of the vector inside b.
 */
__kernel void complexMultiplier(__global float2 *a, __global float2 *b,
                                __global float2 *c,
                                unsigned int points_per_group,
                                unsigned int length, unsigned int offset,
                                unsigned int b_base) {
  uint points_per_item, addr, i, j;

  points_per_item = points_per_group / get_local_size(0);
  addr = get_group_id(0) * points_per_group + get_local_id(0) * points_per_item;

  for (i = addr; i < addr + points_per_item; i++) {
    j = i + offset;
    if (j >= length) {
      j -= length;
    }
    c[i].x = (a[i].x * b[b_base + j].x) - (a[i].y * b[b_base + j].y);
    c[i].y = (a[i].y * b[b_base + j].x) + (a[i].x * b[b_base + j].y);
  }
}

/**
 * @brief Batched complex multiplication for the Doppler-bin search: one launch
 * covers every frequency bin. Work-item i belongs to bin (i / length) and
 * computes c[i] = a[i mod length] * b_pool[base + ((j + offset) mod length)],
 * where base and offset come from that bin's entry in bin_params (x = element
 * offset of the bin's reference spectrum inside b_pool, y = circular shift
 * offset). The output c holds the bins' products back to back, ready for a
 * batched one-workgroup-per-bin FFT.
 *
 * @param a shared first operand, one length-sized complex vector.
 * @param b_pool pool of reference spectra, length-sized vectors back to back.
 * @param c output pool, bins * length complex values.
 * @param bin_params per-bin (pool element offset, circular read offset).
 * @param length element count of one vector.
 */
__kernel void complexMultiplierBatch(__global float2 *a,
                                     __global float2 *b_pool,
                                     __global float2 *c,
                                     __global uint2 *bin_params,
                                     unsigned int length) {
  const uint i = get_global_id(0);
  const uint bin = i / length;
  const uint j = i - (bin * length);
  const uint2 params = bin_params[bin];

  uint k = j + params.y;
  if (k >= length) {
    k -= length;
  }
  const uint src = params.x + k;

  c[i].x = (a[j].x * b_pool[src].x) - (a[j].y * b_pool[src].y);
  c[i].y = (a[j].y * b_pool[src].x) + (a[j].x * b_pool[src].y);
}

/**
 * @brief Computes magnitude squared of complex numbers: c = Re^2 + Im^2
 */
__kernel void absolute(__global float2 *a, __global float *c,
                       unsigned int points_per_group) {
  uint points_per_item, addr, i;

  points_per_item = points_per_group / get_local_size(0);
  addr = get_group_id(0) * points_per_group + get_local_id(0) * points_per_item;

  for (i = addr; i < addr + points_per_item; i++) {
    c[i] = (a[i].x * a[i].x) + (a[i].y * a[i].y);
  }
}

}
