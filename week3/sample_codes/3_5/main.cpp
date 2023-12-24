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
  void InitializeGivenGraph(const unsigned long int sz){
    
    if(sz==5){
      int graphInt[size][size] = {{0, 1, 1, 0, 0},
			   {1, 0, 1, 1, 0},
			   {1, 1, 0, 1, 1},
			   {0, 1, 1, 0, 1},
			   {0, 0, 1, 1, 0}};
      for(unsigned long int i=0;i<size;++i)
	for(unsigned long int j=i;j<size;++j){
	  graph[i][j] = static_cast<bool>(graphInt[i][j]);
	}
    }else if(sz==7){
      int graphInt[size][size] = {{0, 1, 1, 1, 1, 1, 0},
      				  {1, 0, 1, 1, 1, 1, 0},
				  {1, 1, 0, 1, 1, 1, 0},
				  {1, 1, 1, 0, 1, 1, 0},
				  {1, 1, 1, 1, 0, 1, 0},
				  {1, 1, 1, 1, 1, 0, 0},
				  {0, 0, 0, 0, 0, 0, 0}};
      for(unsigned long int i=0;i<size;++i)
      	for(unsigned long int j=i;j<size;++j){
	  graph[i][j] = static_cast<bool>(graphInt[i][j]);
	}
    }
  }
  bool isConnected()const{
    
    unsigned long int old_size{0}, c_size{0};
    bool *close = new bool[size];
    bool *open = new bool[size];
    for(unsigned long int i=0;i<size;++i)
      close[i]=open[i]=false;
    open[0]=true;
    
    for(unsigned long int i=0;i<size;++i){
      old_size = c_size;
      
      if(open[i] && (close[i]==false)){
	close[i] = true;
	++c_size;
      }
      
      for(unsigned long int j=0;j<size;++j){
	open[j] = open[j] || graph[i][j];
      }
      if(c_size==size){
	delete [] close;
        delete [] open;
	return true;
      }
      else if(old_size==c_size){
	break;
      }
    }
    delete [] close;
    delete [] open;
    return false;
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
  
  RandomGraph graph5(5);
  RandomGraph graph7(7);
  graph5.InitializeGivenGraph(5);
  graph5.VisualizeGraph();
  if(graph5.isConnected())
    std::cout<<"The graph is connected.\n";
  else
    std::cout<<"The graph is not connected.\n";
  std::cout<<"\n\n";
  graph7.InitializeGivenGraph(7);
  graph7.VisualizeGraph();
  if(graph7.isConnected())
    std::cout<<"The graph is connected.\n";
  else
    std::cout<<"The graph is not connected.\n";
  
  return 0;
}
