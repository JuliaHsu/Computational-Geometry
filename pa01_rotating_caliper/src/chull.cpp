//------------------------------------------------------------------------------
//  Copyright 2007-2019 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "chull.h"

namespace masc{
namespace polygon{

///////////////////////////////////////////////////////////////////////////////
#include <cassert>
using namespace std;


inline float isLeft(Point2d P0, Point2d P1,Point2d P2 ){
  return (P1[0] - P0[0])*(P2[1] - P0[1]) - (P2[0] - P0[0])*(P1[1] - P0[1]);

}

//
//s and e are the start and the end vertices of the polygon
//e mush be reachable from s
//
void hull2d(ply_vertex * s, ply_vertex * e, list<ply_vertex*>& hull )
{
  
  
  ply_vertex* v= s->getNext();
  // n = total number of vertices
  // auto & pos=v->getPos();
  // cout<<  pos;
  int n = 1;
  while(v!=s){
    n++;
    v=v->getNext();
  }
 

  // initialize a deque D[] from bottom to top so that the

  ply_vertex** D = new ply_vertex*[2*n+1];
  int bot = n-2, top = bot+3;
  D[bot] = D[top] = s->getNext()->getNext();
  auto & P0 =s->getPos();
  auto & P1 =s->getNext()->getPos();
  auto & P2 = s->getNext()->getNext()->getPos();
  if (isLeft(P0, P1, P2) > 0) {
    D[bot+1] = s;
    D[bot+2] = s->getNext();           // ccw vertices are: 2,0,1,2
  }
  else {
    D[bot+1] = s->getNext();
    D[bot+2] = s;          // ccw vertices are: 2,1,0,2
  }
  // compute the hull on the deque D[]
  v = s->getNext()->getNext();
  auto & Pt = v->getPos();
  
  for(int i = 3; i<n;i++){
    v = v->getNext();
    auto & Pt = v->getPos();
    if(isLeft(D[bot]->getPos(),D[bot+1]->getPos(),Pt)>0 && 
    isLeft(D[top-1]->getPos(),D[top]->getPos(),Pt)>0){
      continue;
    }
    while(isLeft(D[bot]->getPos(),D[bot+1]->getPos(),Pt)<=0){
      ++bot;
    }
    D[--bot] = v;
    while(isLeft(D[top-1]->getPos(),D[top]->getPos(),Pt)<=0){
      --top;
    }
    D[++top] = v;
    
    // for(int k =0; k<=(top-bot);k++){
    //   cout<<D[bot+k]->getPos()<<"\n";
    // }
    
  }
  int h;
  
  for(h=0;h<=(top-bot);h++){
    hull.push_back(D[bot+h]);
  }
  

  

 

  
}

}//end namespace polygon
}//end namespace masc
