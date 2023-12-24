#include <iostream>

class listElement{
public:
  int d;
  listElement *next;
  listElement(const int dIn=0, listElement *ptr=nullptr)
    :d(dIn), next(ptr){}
};

class list{
public:
  // Default constructor
  list()
    :head(nullptr), cursor(nullptr){
    std::cout<<"\n";
    std::cout<<"Constructor\n";
  }
  // Deep copy constructor
  list(const list& src){
    std::cout<<"\nCopy constructor\n";
    if(src.head==nullptr){
      head=nullptr;
      cursor=nullptr;
    }else{
      head = new listElement();
      listElement *src_h = src.head;
      listElement *h = head; 
      head->d = src.head->d;
      if(src_h==src.cursor){
	cursor = h;
	cursor->d = h->d;
      }
      while(src_h->next!=nullptr){
      	src_h = src_h->next;
	listElement *hNewew = new listElement();
	h->next=hNewew;
	hNewew->d=src_h->d;
	if(src_h==src.cursor){
	  cursor = hNewew;
          cursor->d = hNewew->d;
	}
	h = hNewew;
      }
      h->next = nullptr;
    }
    std::cout<<"Test of the deep copy constructor:\n";
    this->printHeadCursor();
    this->print();
    std::cout<<"\n";
  }
  // insert a data to the list
  void prepend(const int n){
    if(head==nullptr) // empty list
      cursor = head = new listElement(n,head);
    else
      head = new listElement(n,head);
  }
  int getElement()const{return cursor->d;}
  void advance(){cursor=cursor->next;}
  // print the linked list
  void print()const{
    listElement *h = head;
    std::cout<<"Date is :\n";
    while(h!=nullptr){
      std::cout<<h->d<<",";
      h = h->next;
    }
    std::cout<<"###\n";
  }
  // print the data pointed by head and cursor pointers 
  void printHeadCursor()const{
    std::cout<<"The head is pointing to "<<head->d<<"\n";
    std::cout<<"The cursor is pointing to "<<cursor->d<<"\n";
  }
  //Destructor
  ~list(){
    std::cout<<"\n";
    std::cout<<"Destructor\n";
    do{
      cursor = head->next;
      delete head;
      head = cursor;
    }while(head!=nullptr);
  }
private:
  listElement *head;
  listElement *cursor;
};

void doSomethin(list test){} // Test the deep copy constructor

int main(int argc,char **argv){

  list listTest;
  listTest.prepend(3);
  listTest.prepend(7);
  listTest.prepend(5);
  listTest.printHeadCursor();
  listTest.print();
  doSomethin(listTest);
  return 0;
}
