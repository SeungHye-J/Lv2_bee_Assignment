// cuvslam_slam C++판 GPU 입력 — cuvslam_gpu.cu
#pragma once

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>

// depth(uint16, 행 dpitch 바이트) → mask(uint8, 행 mpitch): 0 < depth ≤ lim이면 1 (추적에서 뺌). 둘 다 장치 메모리
cudaError_t near_mask_gpu(const void * depth, size_t dpitch, void * mask, size_t mpitch, int w, int h, uint16_t lim,
                          cudaStream_t s);
