#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

// export PATH=$PATH:/usr/local/cuda/bin

// CUDA kernel function to set 1 to all values in the float array
__global__
void setOnes(float * my_array) {

  int array_idx = blockIdx.x;
  // int tmp = threadIdx.x;

  my_array[array_idx] = 1;
}

int main(int argc, char * argv[]) {

    float * cpu_zero_array = new float[100];
    memset(cpu_zero_array, 0, sizeof(float) * 100);

    float * gpu_zero_array;
    cudaMalloc(&gpu_zero_array, 100 * sizeof(float));
    cudaMemcpy(gpu_zero_array, cpu_zero_array, 100 * sizeof(float), cudaMemcpyHostToDevice);
    setOnes<<<100,1>>>(gpu_zero_array);
    cudaMemcpy(cpu_zero_array, gpu_zero_array, 100 * sizeof(float), cudaMemcpyDeviceToHost);

    for (int i = 0; i < 100; ++i)
        std::cout << cpu_zero_array[i] << std::endl;

}

























