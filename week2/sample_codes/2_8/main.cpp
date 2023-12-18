#include <iostream>

typedef enum days{SUNDAY,MONDAY,TUESDAY,WEDNESDAY,THURSDAY,FRIDAY,SATURDAY}days;

//pre-increment
inline days operator ++(days d){return static_cast<days>((static_cast<int>(d)+1)%7);}

//operator overloading
std::ostream& operator << (std::ostream &out, const days d){
  switch(d){
  case SUNDAY:out<<"Sunday";break;
  case MONDAY:out<<"Monday";break;
  case TUESDAY:out<<"Tuesday";break;
  case WEDNESDAY:out<<"Wednsday";break;
  case THURSDAY:out<<"Thursday";break;
  case FRIDAY:out<<"Friday";break;
  case SATURDAY:out<<"Saturday";break;
  default:out<<"Invalid day";break;
  }
  return out;
}


int main(int argc, char **argv){
  days myday=WEDNESDAY;
  std::cout<<++myday<<"\n";
  return 0;
}
