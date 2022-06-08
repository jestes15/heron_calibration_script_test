#include <algorithm>
#include <chrono>
#include <execution>
#include <iostream>
#include <vector>

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/algorithm/cxx17/for_each_n.hpp>

#include "Fusion.h"
#include "vector_ext.hpp"

struct fib_vector
{
    std::vector<boost::multiprecision::cpp_int> fib{0};
} vector_holder;

boost::multiprecision::cpp_int fibonacci(long long n)
{
    if (vector_holder.fib.size() > n)
    {
        return vector_holder.fib.at(n);
    }
    if (n == 0)
    {
        vector_holder.fib.push_back(0);
        return 0;
    }
    if (n == 1)
    {
        vector_holder.fib.push_back(1);
        return 1;
    }
    vector_holder.fib.push_back(fibonacci(n - 1) + fibonacci(n - 2));
    return vector_holder.fib.at(n);
}

template <typename T>
auto print(std::vector<T> &vec)
{
    std::cout << "{ ";
    for (auto i : vec)
        std::cout << i << " ";
    std::cout << "}\n";
}

int main()
{
    long long fib_num;
    std::cout << "Fibonacci calc: ";
    std::cin >> fib_num;

    std::cout << "Running fibonacci(" << fib_num << ")\n";
    boost::multiprecision::cpp_int start_time_non_cache = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    boost::multiprecision::cpp_int ret = fibonacci(fib_num);
    boost::multiprecision::cpp_int end_time_non_cache = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::cout << "Fibonacci norm ret val: " << ret << ", Execution time: " << end_time_non_cache - start_time_non_cache << " ms\n";


    std::cout << "Generating Quaternion\n";
    FusionQuaternion quaternion = {
        .element = {.w = 0, .x = 1, .y = 2, .z = 3},
    };

    std::cout << "w: " << quaternion.array[0] << std::endl;
    std::cout << "x: " << quaternion.array[1] << std::endl;
    std::cout << "y: " << quaternion.array[2] << std::endl;
    std::cout << "z: " << quaternion.array[3] << std::endl;

    std::cout << "Generatig Vector\n";
    FusionVector vector = {
        .axis = { .x = 0, .y = 1, .z = 2 }
    };

    std::cout << "x: " << vector.array[0] << std::endl;
    std::cout << "y: " << vector.array[1] << std::endl;
    std::cout << "z: " << vector.array[2] << std::endl;

    std::cout << "Generating FusionMatrix\n";
    FusionMatrix matrix = {
        .element ={ 
            .xx = 0, .xy = 1, .xz = 2,
            .yx = 0, .yy = 1, .yz = 2,
            .zx = 0, .zy = 1, .zz = 2
        }
    };

    std::cout << "xx: " << matrix.array[0][0] << std::endl;
    std::cout << "xy: " << matrix.array[0][1] << std::endl;
    std::cout << "xz: " << matrix.array[0][2] << std::endl;

    std::cout << "yx: " << matrix.array[1][0] << std::endl;
    std::cout << "yy: " << matrix.array[1][1] << std::endl;
    std::cout << "yz: " << matrix.array[1][2] << std::endl;

    std::cout << "zx: " << matrix.array[2][0] << std::endl;
    std::cout << "zy: " << matrix.array[2][1] << std::endl;
    std::cout << "zz: " << matrix.array[2][2] << std::endl;

    std::cout << "Generating FusionEuler\n";
    FusionEuler euler = {
        .angle = {
            .roll = 0,
            .pitch = 1,
            .yaw = 2,
        }
    };

    std::cout << "Roll: " << euler.array[0] << std::endl;
    std::cout << "Pitch: " << euler.array[1] << std::endl;
    std::cout << "Yaw: " << euler.array[2] << std::endl;

    return 0;
}