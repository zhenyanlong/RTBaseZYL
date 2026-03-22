#pragma once

#include <iostream>
#include <vector>

namespace Debug
{
	template<typename T>
	void printVector(const T& vec)
	{
		std::cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")" << std::endl;
	}

	template<typename T>
	void printArray(const std::vector<T>& array)
	{
		for (size_t i = 0; i < array.size(); i++)
		{
			std::cout << "Element " << i << ": " << array[i] << std::endl;

		}
	}
}
