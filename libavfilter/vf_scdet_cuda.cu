__device__ inline unsigned int reduce_warp(unsigned int val)
{
  for (int offset = 16; offset > 0; offset /= 2)
    val += __shfl_down_sync(0xffffffff, val, offset);

  return val;
}

__device__ inline unsigned int sumdiff_uint8(unsigned int a, unsigned int b)
{
  unsigned int sum = __vabsdiffu4(a, b);
  return (sum & 0x000000FF) + ((sum & 0x0000FF00) >> 8) + ((sum & 0x00FF0000) >> 16) + ((sum & 0xFF000000) >> 24);
}
__device__ inline unsigned int sumdiff_uint16(unsigned int a, unsigned int b)
{
  unsigned int sum = __vabsdiffu2(a, b);
  return (sum & 0x0000FFFF) + ((sum & 0xFFFF0000) >> 16);
}

__device__ void reduce(unsigned int *sums, unsigned int *sdata, cudaTextureObject_t src1, cudaTextureObject_t src2, unsigned int (*sumdiff)(unsigned int, unsigned int))
{
  float col_index = blockIdx.x * blockDim.x + threadIdx.x;
  float row_index = blockIdx.y * blockDim.y + threadIdx.y;
  const int tid = threadIdx.y * blockDim.x + threadIdx.x;
  const int bid = blockIdx.y * gridDim.x + blockIdx.x;

  // reads eight bytes from each source (four shorts)
  ushort4 data1 = tex2D<ushort4>(src1, col_index, row_index); 
  ushort4 data2 = tex2D<ushort4>(src2, col_index, row_index); 
  // treat the data as u32 so that we can perform vector operations
  unsigned int *data1_ptr = reinterpret_cast<unsigned int *>(&data1);
  unsigned int *data2_ptr = reinterpret_cast<unsigned int *>(&data2);
  
  unsigned int sad = sumdiff(data1_ptr[0], data2_ptr[0]); //compare the first 4 bytes (or 2 shorts)
  sad += sumdiff(data1_ptr[1], data2_ptr[1]); //compare the last 4 bytes (or 2 shorts)

  __syncthreads();
  unsigned int sum = reduce_warp(sad);

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

extern "C"
{
    __global__ void scdet_8(unsigned int *sums, cudaTextureObject_t src1, cudaTextureObject_t src2)
    {
        extern __shared__ unsigned int sdata[];
        reduce(sums, sdata, src1, src2, sumdiff_uint8);
    }

    __global__ void scdet_16(unsigned int *sums, cudaTextureObject_t src1, cudaTextureObject_t src2)
    {
        extern __shared__ unsigned int sdata[];
        reduce(sums, sdata, src1, src2, sumdiff_uint16);
    }
}