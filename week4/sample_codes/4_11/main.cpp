#include <iostream>
#include <vector>
#include <algorithm>

int main(int argc, char *argv[]){

  int size{5};
  std::vector<double> data(size);
  std::cout<<"Please enter "<<size<<" floating-point numbers to store in vector container\n";
  for(auto iter = data.begin();iter!=data.end();++iter)
    std::cin>>*iter;

  std::cout<<"The entered numbers are : \n";
  for(auto element : data)
    std::cout<<element<<"\n";
  return 0;
}
