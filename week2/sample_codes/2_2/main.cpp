#include <iostream>

template<typename T1, typename T2>
void copy(const T1 source[], T2 destinaion[], int size){
  for(int i=0;i<size;++i){destinaion[i]= static_cast<T2> (source[i]);}
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
  printArray(b,3);
  std::cout<<"Convert the double array to integer:\n";
  int bInt[3];
  copy(b,bInt,3);
  printArray(bInt,3);
  return 0;
}
