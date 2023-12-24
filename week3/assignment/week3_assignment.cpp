/*Homework 2: Implement Dijkstra's Algorithm with OOP for week3 of C++ For C Programmers, Part A
 The adjacency matrix has been used to present the undirected graph, and
 the adjacency matrix has been used for the Dijkstra's Algorithm (short pass tree).  
 The graph is produced by using random numbers with given the number of
 vortices, edge density, and a limit for distance between vortices.
 
 Refernces:

 https://www.geeksforgeeks.org/graph-data-structure-and-algorithms/
 https://www.geeksforgeeks.org/generate-a-random-float-number-in-cpp/?ref=header_search
 
 Kazem Bazesefidpar December 24, 2023
*/

#include <iostream>
#include <vector>
#include <random>
#include <limits>

const double inf = std::numeric_limits<double>::infinity();

class Random{
private:
  int a, b;
public:
  Random(const int aIn=0,const int bIn=1)
    :a(aIn),b(bIn){
    srand(time(0));
  }
  // compute a double random number
  double randomDoub(){return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);}
  // Compute a integer randon number in [a,b] where b>a
  int randomInt(){return a + (rand() % (b - a));}
  // Compute a double randon number in [a,b] where b>a
  double randomDouble(){return static_cast<double>(randomInt()) + randomDoub();}
};
// struct used to store the graph's vortices and cost
typedef struct{
  bool edge; // if two nodes are connected, egde=true
  double cost; // distance which is a positive value
}VE;
// struct used in Dijkstra's algorithm
typedef struct{
  bool spt;     // true if the vortex is in the shortest pass  
  double dist;  // the shortest distance from source to ith vortex
}set;

// main class
class Graph{
private:
  VE **graphRandom;          
  unsigned long int size;             // size of graph
  int lowBound,upperBound;            // The lower and upper bound for the distance between vertices
  double density;                    // edge density
  double distMin;                    // The minimum distance for a given destination 
  set *DijkstraAlg;                   /* The struct to store the  short pass tree (spt) and distance (dist)
					in Dijkstra algorithm for a given destination; it will be initialize 
					in each call to Dijkstra algorithm with a new destination*/
  int desti;                         // distance vortex in each search
public:
  Graph(unsigned long int sz,int lowBoundIn,int upperBoundIn,double denIn)
    :size(sz),lowBound(lowBoundIn),upperBound(upperBoundIn),density(denIn),distMin(0),desti(0){
    /*allocate memory for the graph
     VE is a struct with two elements*/
    graphRandom = new VE*[size];
    for(unsigned long int i=0;i<size;++i)
      graphRandom[i] = new VE[size];
    // allocate memory for set
    DijkstraAlg = new set[size];
    // create two random object using the Random class
    Random prob, number(lowBound,upperBound);
    
    /*//A graph can be added here instead of the random graph genertion
     //for testing purpose. In this case, comment the lines of the code
     // for the generation of the random graph.
     // size should be 9 (size=9) in the main for this graph
     double gra[size][size] = { { 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0 },
			       { 4.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 11.0, 0.0 },
			       { 0.0, 8.0, 0.0, 7.0, 0.0, 4.0, 0.0, 0.0, 2.0 },
			       { 0.0, 0.0, 7.0, 0.0, 9.0, 14.0, 0.0, 0.0, 0.0 },
			       { 0.0, 0.0, 0.0, 9.0, 0.0, 10.0, 0.0, 0.0, 0.0 },
			       { 0.0, 0.0, 4.0, 14.0, 10.0, 0.0, 2.0, 0.0, 0.0 },
			       { 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 1.0, 6.0 },
			       { 8.0, 11.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 7.0 },
			       { 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 6.0, 7.0, 0.0 } };
    for(unsigned long int i=0;i<size;++i){  
      for(unsigned long int j=0;j<size;++j){
	graphRandom[i][j].edge = static_cast<bool>(gra[i][j]);
    	graphRandom[i][j].cost = gra[i][j];
     }
    }
    */

    // initialize the random graph
    for(unsigned long int i=0;i<size;++i){
      for(unsigned long int j=i;j<size;++j){
    	if(i==j){
    	  graphRandom[i][j].edge = false; // no loop
    	  graphRandom[i][j].cost = 0.0;
    	}else{
    	  graphRandom[i][j].edge = graphRandom[j][i].edge = static_cast<bool>(prob.randomDouble()<density);
    	  graphRandom[i][j].cost = graphRandom[j][i].cost = number.randomDouble()*(static_cast<int>(graphRandom[i][j].edge));
    	}
     }
    }
  }
  
  bool isConnected(const int searchNod)const{
    
    unsigned long int old_size{0}, c_size{0};
    bool *close = new bool[size];
    bool *open = new bool[size];
    
    for(unsigned long int i=0;i<size;++i)
      close[i]=open[i]=false;
    open[0]=true; // search starts from node 0
    
    for(unsigned long int i=0;i<size;++i){
      old_size = c_size;

      if(open[i] && (close[i]==false)){
        close[i] = true;
        ++c_size;
      }
      
      for(unsigned long int j=0;j<size;++j){
        open[j] = open[j] || graphRandom[i][j].edge;
      }
      if(c_size==size || open[searchNod]==true){ /* if node searchNod is in the open set, the 
						    node 0 is connected to searchNod so that search is terminated.*/
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
  // algorithm to compute the shortest pass between src and desti vortices by dijkstra algorithm
  void dijkstra(const int src,const int destiIn){
    
    distMin = 0.0; // initialize in each search to zero
    desti = destiIn;
    for (unsigned long int i = 0; i < size; ++i){
      DijkstraAlg[i].dist = inf;
      DijkstraAlg[i].spt = false;
    }
    
    DijkstraAlg[src].dist = 0; // initilize src to zero
    //std::cout<<"\nThe desti is "<<destiIn<<"\n\n";
    int MinDisVortSelct{0};
    double minimum;
    for (unsigned long int count = 0; count < size-1; count++) {
      
      // find minimum distance vertex from not selected vortices
      minimum=inf; // set the initial minimum distance to infinity
      for (unsigned long int i = 0; i < size; ++i){
  	if (DijkstraAlg[i].spt==false && DijkstraAlg[i].dist <= minimum){
  	  minimum = DijkstraAlg[i].dist;
  	  MinDisVortSelct = i;
  	}
      }
      // put the vortex with minimum disance from the temperry source in the selected set
      DijkstraAlg[MinDisVortSelct].spt = true;
      //std::cout<<MinDisVortSelct<<" is inserted in the minimum path with "<<DijkstraAlg[MinDisVortSelct].dist<<"\n";
      
      // Update dist value of the adjacent vertices of the
      // picked vertex.
      for (unsigned long int i = 0; i < size; ++i)	
  	if (!DijkstraAlg[i].spt && graphRandom[MinDisVortSelct][i].edge
  	    && DijkstraAlg[MinDisVortSelct].dist != inf
  	    && DijkstraAlg[MinDisVortSelct].dist + graphRandom[MinDisVortSelct][i].cost < DijkstraAlg[i].dist){
  	  DijkstraAlg[i].dist = DijkstraAlg[MinDisVortSelct].dist + graphRandom[MinDisVortSelct][i].cost;
	  //std::cout<<i<<" is in the neihgbor with "<<DijkstraAlg[i].dist<<"\n";
  	}
      // exit the loop if the current vortex is the destination node
      if(MinDisVortSelct==desti){
        distMin=DijkstraAlg[MinDisVortSelct].dist;
        break;
      }
    }
  }
  // get the minimum distance in each search
  double getMinDistValue()const{return distMin;}
  //visulize the adjacency matrix
  void VisualizeGraph(const char name)const{
    std::cout<<"\t";
    for(unsigned long int i=0;i<size;++i)
      std::cout<<i<<"\t";
    std::cout<<"\n\n";
    for(unsigned long int i=0;i<size;++i){
      std::cout<<i<<"\t";
      for(unsigned long int j=0;j<size;++j){
	if(name=='E') // plot the edge 1 or 0
	  std::cout<<graphRandom[i][j].edge<<"\t";
	else if (name=='D') // plot the cost between two vortices
	  std::cout<<graphRandom[i][j].cost<<"\t";
      }
      std::cout<<"\n";
    }
  }
  // plot the SPT in each search with the corresponding distance
  void plotShort()const{
    // we stored nodes in the SPT and distances in two vectors
    std::vector<int> path;
    std::vector<double> PathDist;
    int node{desti},nextNode{0};
    
    // push the destination node and its distance into the vectors
    path.push_back(node);
    PathDist.push_back(DijkstraAlg[node].dist);
    //std::cout<<"\n in graph plot "<<node<<"\n";
    
    do{
      
      for(unsigned long int i=0;i<size;++i){ /* iterate to find the vortex in 
						the adjecency of (the node) in the SPT*/
	
	if(DijkstraAlg[i].spt==true && graphRandom[node][i].edge==true
	   && DijkstraAlg[i].dist!=inf
	   && DijkstraAlg[node].dist == DijkstraAlg[i].dist + graphRandom[i][node].cost
	   && std::find(path.begin(),path.end(),i)==path.end()){ // algorithm to skip already selected neighbours
	  
	  nextNode = i;
	  break;
	}
      }
      path.push_back(nextNode);
      PathDist.push_back(DijkstraAlg[nextNode].dist);
      node = nextNode;
      
    }while(DijkstraAlg[node].dist!=0); // terminate when reaches to the src
    
    for(int i=path.size()-1;i>0;--i)
      if(i>1)
    	std::cout<<path[i]<<" --- (cost="<<PathDist[i-1]<<") --->"<<" ";
      else
    	std::cout<<path[i]<<" --- (cost="<<PathDist[i-1]<<") ---> "<<path[i-1]<<"\n";
  }
  ~Graph(){
    //std::cout<<"Destructor\n";
    for(unsigned long int i=0;i<size;++i)
      delete [] graphRandom[i];
    delete [] graphRandom;
    delete [] DijkstraAlg;
  }
};


int main(int argc,char **argv){
  
  unsigned long int size{50};
  double density{0.2};
  double a{0.0}, b{10.0}; // the range of distance between vortices
  int count{0};
  double minDistance{0};
  Graph graph20(size,a,b,density);
  // visualize the graph E : edge (0 or 1) ; D : distance between vortices
  //graph20.VisualizeGraph('E');
  //graph20.VisualizeGraph('D');
  std::cout<<"\nThe graph with density "<<density<<"\n";
  for(unsigned long int i=1;i<size;++i){
    if(graph20.isConnected(i)==true){
      graph20.dijkstra(0,i);
      graph20.plotShort();
      minDistance+= graph20.getMinDistValue();
      count+=1;
    }
  }
  if(count!=0)
    std::cout<<"\n\nThe average path length for graph with density "<<density<<" is "<<minDistance/count<<"\n";
  else
    std::cout<<"\n\nThe graph is not conneced\n";
  // The computation for density 40
  density=0.4;
  minDistance=0;
  count=0;
  Graph graph40(size,a,b,density);
  std::cout<<"\nThe graph with density "<<density<<"\n";
  for(unsigned long int i=1;i<size;++i){
    if(graph40.isConnected(i)==true){
      graph40.dijkstra(0,i);
      graph40.plotShort();
      minDistance+= graph40.getMinDistValue();
      count+=1;
    }
  }
  if(count!=0)
    std::cout<<"\n\nThe average path length for graph with density "<<density<<" is "<<minDistance/count<<"\n";
  else
    std::cout<<"\n\nThe graph is not conneced\n";
  
  return 0;
}
