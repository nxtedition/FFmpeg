__device__ inline unsigned int reduce_warp(unsigned int val)
{
    for (int offset = 16; offset > 0; offset /= 2)
        val += __shfl_down_sync(0xffffffff, val, offset);

    return val;
}

__device__ inline int count_uint8(unsigned int data, unsigned int packed_limit)
{
    unsigned int cmp = __vadd4(__vcmpgeu4(data, packed_limit), 0x01010101);
    return __popc(cmp);
}
__device__ inline int count_uint16(unsigned int data, unsigned int packed_limit)
{
    unsigned int cmp = __vadd2(__vcmpgeu2(data, packed_limit), 0x00010001);
    return __popc(cmp);
}

__device__ void reduce(unsigned int *sums, unsigned int *sdata, unsigned int packed_limit, unsigned int gridDim_x, cudaTextureObject_t texObj, int (*counter)(unsigned int, unsigned int))
{
    const int tid = threadIdx.y * blockDim.x + threadIdx.x;
    const int bid = blockIdx.y * gridDim_x + blockIdx.x;
    float col_index = blockIdx.x * blockDim.x + threadIdx.x;
    float row_index = blockIdx.y * blockDim.y + threadIdx.y;

    ushort4 data = tex2D<ushort4>(texObj, col_index, row_index);
    unsigned int *data_ptr = reinterpret_cast<unsigned int *>(&data);
    unsigned int count = counter(data_ptr[0], packed_limit);
    count += counter(data_ptr[1], packed_limit);

    __syncthreads();
    unsigned int sum = reduce_warp(count);

    if ((tid % 32) == 0)
        sdata[tid / 32] = sum;

    __syncthreads();

    // reduce the entire block in the first warp
    if (tid < 32)
    {
        unsigned int val = (tid < (blockDim.x * blockDim.y / 32)) ? sdata[tid] : 0;
        sum = reduce_warp(val);

        // write result for this block to global mem
        if (tid == 0)
            sums[bid] = sum;
    }
}

extern "C" {
__global__ void blackdetect_8(unsigned int *sums, unsigned int packed_limit, unsigned int gridDim_x, cudaTextureObject_t texObj)
{
    extern __attribute__((shared)) unsigned int sdata[];
    reduce(sums, sdata, packed_limit, gridDim_x, texObj, count_uint8);
}

__global__ void blackdetect_16(unsigned int *sums, unsigned int packed_limit, unsigned int gridDim_x, cudaTextureObject_t texObj)
{
    extern __attribute__((shared)) unsigned int sdata[];
    reduce(sums, sdata, packed_limit, gridDim_x, texObj, count_uint16);
}
}