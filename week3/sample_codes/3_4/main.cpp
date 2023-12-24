#include <iostream>
#include <random>

class RandomGraph{
private:
  bool **graph;
  unsigned long int size;
public:
  RandomGraph(const unsigned long int sz)
    :size(sz){
    graph	= new bool*[size];
    for(unsigned long int i=0;i<size;++i)
      graph[i] = new bool[size];
  }
  // Generate a random number between a and b
  double prob(const double a,const double b){

    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> probability(a,b);
    double pb = probability(generator);
    return pb;
    
  }
  // Generte a random graph with given probability with conectivity of 0 or 1
  void InitializeRandomGraph(const double probability,const double a = 0.0, const double b = 1.0){

    for(unsigned long int i=0;i<size;++i)
      for(unsigned long int j=i;j<size;++j){
	if(i==j)
	  graph[i][j] = false; // no loop
	else
	  graph[i][j] = graph[j][i] = static_cast<bool>(prob(a,b)<probability);
      }
	
  }
  void VisualizeGraph()const{
    std::cout<<"\t";
    for(unsigned long int i=0;i<size;++i)
      std::cout<<i<<"\t";
    std::cout<<"\n\n";
    for(unsigned long int i=0;i<size;++i){
      std::cout<<i<<"\t";
      for(unsigned long int j=0;j<size;++j){
	std::cout<<graph[i][j]<<"\t";
      }
      std::cout<<"\n";
    }
  }
  ~RandomGraph(){
    for(unsigned long int i=0;i<size;++i)
      delete [] graph[i];
    delete [] graph;
  }
};


int main(int argc,char **argv){
  const unsigned long int size=10;
  double probability = 0.19;
  double a{0.0}, b{1.0}; // the range of random number
  RandomGraph graph(size);
  graph.InitializeRandomGraph(probability,a,b);
  graph.VisualizeGraph();
  return 0;
}
