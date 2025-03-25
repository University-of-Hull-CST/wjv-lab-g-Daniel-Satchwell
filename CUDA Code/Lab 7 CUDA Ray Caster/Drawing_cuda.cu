/* Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BICUBICTEXTURE_CU_
#define _BICUBICTEXTURE_CU_


#include <helper_gl.h>
#include <GL/freeglut.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <helper_math.h>

 // includes, cuda
#include <helper_cuda.h>

//header files for raycasting from Ray Tracing In One Weekend
#include <hitable.h>
#include <hitable_list.h>
#include <ray.h>
#include <sphere.h>
#include <vec3.h>

typedef unsigned int uint;
typedef unsigned char uchar;


cudaArray* d_imageArray = 0;

void check_cuda(cudaError_t result, char const* const func, const char* const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " <<
            file << ":" << line << " '" << func << "' \n";
        // Make sure we call CUDA Device Reset before exiting
        cudaDeviceReset();
        exit(99);
    }
}
__device__ vec3 castRay(const ray& r, hitable** world) {
    hit_record rec;
    if ((*world)->hit(r, 0.0, FLT_MAX, rec)) {
        return 0.5f * vec3(rec.normal.x() + 1.0f, rec.normal.y() + 1.0f, rec.normal.z() + 1.0f);
    }
    else {
        vec3 unit_direction = unit_vector(r.direction());
        float t = 0.5f * (unit_direction.y() + 1.0f);
        return (1.0f - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);
    }
}
__global__ void create_world(hitable** d_list, hitable** d_world) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        int val = 12;
        float x = -4;
        float y = 0;
        //*(d_list) = new sphere(vec3(0, 0, -1), 0.5);
    
        for (int i = 0; i < val - 1; i++)
        {
            
            *(d_list + i) = new sphere(vec3(x, y, -1), 0.5);
            
            x = x + 0.5;
         

            if (i % 2 == 0)
            {
                x = x + 1;
                //x = x - 1;
            }
            else
            {
                //x = x / 2;
                //x = x + 1;
                y = y + 0.5;
            }
            
            
        }
        *(d_list + 11) = new sphere(vec3(0, -100.5, -1), 100);
        *d_world = new hitable_list(d_list, val);
    }
}
__global__ void free_world(hitable** d_list, hitable** d_world) {
    delete* (d_list);
    delete* (d_list + 1);
    delete* d_world;
}


__global__ void d_render(uchar4* d_output, uint width, uint height, hitable **d_world) {
    uint x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    uint y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;
    uint i = __umul24(y, width) + x;

    float u = x / (float)width; //---> [0,1] x [0,1]
    float v = y / (float)height;

    u = 2.0f * u - 1.0f; //---> [-1,1] x [-1, 1]
    v = -(2.0f * v - 1.0f);

    u *= width / (float)height;

    u *= 2.0;
    v *= 2.0;

    vec3 eye = vec3(0, 0.5, 1.5);
    float distFrEye2Img = 1.0;;
    if ((x < width) && (y < height))
    {
        //for each pixel
        vec3 pixelPos = vec3(u, v, eye.z() - distFrEye2Img);

        //fire a ray
        ray r;
        r.O = eye;

        r.DIR = pixelPos - eye; //view direction along the negative axis
        
        vec3 col = castRay(r, d_world);
        float red = col.x();
        float green = col.y();
        float blue = col.z();
        d_output[i] = make_uchar4(red * 255, green * 255, blue * 255, 0);
    }

    

    //if ((x < width) && (y < height)) {
    //    /*int cx = width / 2;
    //    int cy = height / 2;
    //    float radius = min(width, height) / 4;*/

    //    float cx = 0.0f;
    //    float cy = 0.0f;
    //    float radius = 0.25f;

    //    float u = x / (float)width;
    //    float v = y / (float)height;

    //    u = 2.0f * u - 1.0f;
    //    v = -(2.0f * v - 1.0f);

    //    u *= width / (float)height;



    //    //float2 z = { u, v };
    //    //float2 T = { 0.1, 0.7 };
    //    //float r = 0;
    //    //float c = 1.0f;
    //    //for (int i = 0; i < 30; i++)
    //    //{
    //    //    //z = { z.x * z.x - z.y * z.y, 2.0f * z.x * z.y };
    //    //    //z += T;

    //    //    float tempX = z.x * z.x - z.y * z.y;
    //    //    float tempY = 2.0f * z.x * z.y;

    //    //    z.x = tempX + T.x;
    //    //    z.y = tempY + T.y;

    //    //    r = sqrt(z.x * z.x + z.y * z.y);
    //    //    if (r > 5.0)
    //    //    {
    //    //        c = 0.0;
    //    //        break;
    //    //    }
    //    //}


    //    //int distanceSquare = ((cx - u) * (cx - u)) + ((cy - v) * (cy - v));
    //    float distanceSquare = (u - cx) * (u - cx) + (v - cy) * (v - cy);

    //    //int c = ((((x & 0x1a) == 0) ^ ((y & 0x1a)) == 0)) * 255;
    //    //if (c == 255)
    //    if (distanceSquare <= radius * radius)
    //    //if (c == 0.0f)
    //    {
    //        d_output[i] = make_uchar4(0, 0, 0xff, 0xff);
    //    }
    //    else
    //    {
    //        d_output[i] = make_uchar4(0, 0, 0, 0);
    //    }

    //}
}


extern "C" void freeTexture() {

    checkCudaErrors(cudaFreeArray(d_imageArray));
}

// render image using CUDA
extern "C" void render(int width, int height, dim3 blockSize, dim3 gridSize,
    uchar4* output) {

    // make our world of hitables
    hitable** d_list;
    checkCudaErrors(cudaMalloc((void**)&d_list, 2 * sizeof(hitable*)));
    hitable** d_world;
    checkCudaErrors(cudaMalloc((void**)&d_world, sizeof(hitable*)));
    create_world << <1, 12 >> > (d_list, d_world);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    d_render << <gridSize, blockSize >> > (output, width, height, d_world);
    getLastCudaError("kernel failed");

    /*d_render << <gridSize, blockSize >> > (output, width, height);


    getLastCudaError("kernel failed");*/
}


#endif