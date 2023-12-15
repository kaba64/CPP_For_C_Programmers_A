#include <iostream>

template<typename T>
T sum(const T data[],int size,T s=0){
  for(int i=0;i<size;++i){s+=data[i];}
  return s;
}

template<typename T>
void printArray(const T &x, int sz){
  if(sizeof(x[0])==8)
    std::cout<<"The array type is double\n";
  else if(sizeof(x[0])==4)
    std::cout<<"The array type is integer\n";
  
  for(int i=0;i<sz;++i)
    std::cout<<"arry["<<i<<"] = "<<x[i]<<"\n";
}

int main(int argc, char *argv[]){
  int a[]={1,2,3};
  double b[] = {2.1,2.2,2.3};
  printArray(a,3);
  std::cout<<"template for sum() = "<<sum(a,3)<<"\n";
  printArray(b,3);
  std::cout<<"template for sum() = "<<sum(b,3)<<"\n";
  return 0;
}
