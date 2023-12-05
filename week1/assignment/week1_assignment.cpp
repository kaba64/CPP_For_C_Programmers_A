// Assignemt for week1 of C++ For C Programmers, Part A
// converting a C code to C++ code
// K. B. December 5, 2023

#include <iostream>
#include <vector>

const int N = 40;

// Use template to make the sum function generic
template<class T>

// sum function should sum the elements of
// the vector d, and store the sum in p variable
inline void sum(T &p, const int n, const std::vector<T> d){
  // use the locality for the variable in the loop
  // the varible p is passed in by reference
  // use the keyword const for safety 
  p=0;
  for(int i=0; i < n; ++i)
    p += d.at(i);
}

int main(void){
  int accum = 0;
  std::vector<int> data;
  //initialize the data
  for(int i = 0; i < N; ++i)
    data.push_back(i);
  sum(accum, N, data);
  std::cout<<"sum is "<<accum<<std::endl;
  return 0;
}
