#ifndef RAYH
#define RAYH
#include "vec3.h"

class ray
{
    public:
        __device__ ray() {}
        __device__ ray(const vec3& a, const vec3& b) { O = a; DIR = b; }
        __device__ vec3 origin() const       { return O; }
        __device__ vec3 direction() const    { return DIR; }
        __device__ vec3 point_at_parameter(float t) const { return O + t*DIR; }

        vec3 O;
        vec3 DIR;
};

#endif
