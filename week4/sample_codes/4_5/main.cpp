#include <iostream>
#include <cmath>

class point{
private:
  double x, y;
public:
  explicit point(double xIn, double yIn=0.0)
    :x(xIn), y(yIn){std::cout<<"Constructor : "<<"("<<x<<","<<y<<")\n";}
  /* not a good idea! */
  //operator double(){
  //  return sqrt(x*x+y*y);
  //}
  point operator + (const point &p){
    std::cout<<"In operator + : "<<"("<<x<<","<<y<<")\n";
    point pout(x+p.x,y+p.y);
    return pout;
  }
  friend std::ostream& operator<<(std::ostream& out, const point& s);
  ~point(){std::cout<<"Destructor : "<<"("<<x<<","<<y<<")\n";}
};

std::ostream& operator<<(std::ostream& out, const point& s){
    out<<"("<<s.x<<","<<s.y<<")";
    return out;
  }

int main(int argc, char *argv[]){

  point s1(3.0,3.0);
  point s2(2.0);
  double d1{6.0};
  /*by using the explicit key word the implicit conversion will generate
   compile error, but static_cast<point> will work */
  //point s3 = d1;
  point s3 = static_cast<point>(d1);
  std::cout<<"s1 = "<<s1<<"\n";
  std::cout<<"s2 = "<<s2<<"\n";
  point s4(0.0,0.0);
  s4 = s1+s2;
  std::cout<<"s1+s2 = "<<s4<<"\n";
  /* invalid conversion unless we use -operator double()- in the
   class, but it is not a good idea*/
  //double d2 = static_cast<double>(s1);
  //std::cout<<d2<<"\n";
  return 0;
}
