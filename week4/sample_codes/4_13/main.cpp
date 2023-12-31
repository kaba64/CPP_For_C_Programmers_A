#include <iostream>
#include <fstream>
#include <numeric>

int main(int argc,char *argv[]){
  
  std::ifstream fin("data.txt");
  if (!fin.is_open()) {
    std::cerr << "Error opening file!\n";
    return (1);
  }
  
  std::istream_iterator<int> fin_it(fin);
  std::istream_iterator<int> end;
  
  std::cout<<"Sum of data is "<<std::accumulate(fin_it,end,0)<<"\n"; // start with sum = 0
  
  fin.close();
  
  return 0;
}
