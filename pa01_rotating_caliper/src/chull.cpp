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
  if ((P1[0] - P0[0])*(P2[1] - P0[1]) - (P2[0] - P0[0])*(P1[1] - P0[1]) > 0) {
    D[bot+1] = s;
    D[bot+2] = s->getNext();           
  }
  else {
    D[bot+1] = s->getNext();
    D[bot+2] = s;       
  }
  // compute the hull on the deque D[]
  v = s->getNext()->getNext();
  auto & Pt = v->getPos();
  Vector3d b1, b2, t1, t2;
  for(int i = 3; i<n;i++){
    v = v->getNext();
    auto & Pt = v->getPos();
    Point2d B0 = D[bot]->getPos();
    Point2d B1 = D[bot+1]->getPos();
    Point2d T0 = D[top-1]->getPos();
    Point2d T1 = D[top]->getPos();

    if(((B1[0] - B0[0])*(Pt[1] - B0[1])-((Pt[0]-B0[0])*(B1[1]-B0[1]))) >0 && ((T1[0]-T0[0])*(Pt[1]-T0[1])-(Pt[0]-T0[0])*(T1[1]-T0[1]))>0){
      continue;
    }
    while(((B1[0] - B0[0])*(Pt[1] - B0[1])-((Pt[0]-B0[0])*(B1[1]-B0[1]))) <=0){
      ++bot;
      B0 = D[bot]->getPos();
      B1 = D[bot+1]->getPos();
    }
     D[--bot] = v;
    while(((T1[0]-T0[0])*(Pt[1]-T0[1])-(Pt[0]-T0[0])*(T1[1]-T0[1]))<=0){
      --top;
      T0 = D[top-1]->getPos();
      T1 = D[top]->getPos();
    }
    D[++top] = v;
    // if(isLeft(D[bot]->getPos(),D[bot+1]->getPos(),Pt)>0 && 
    // isLeft(D[top-1]->getPos(),D[top]->getPos(),Pt)>0){
    //   continue;
    // }
    // while(isLeft(D[bot]->getPos(),D[bot+1]->getPos(),Pt)<=0){
    //   ++bot;
    // }
    // D[--bot] = v;
    // while(isLeft(D[top-1]->getPos(),D[top]->getPos(),Pt)<=0){
    //   --top;
    // }
    // D[++top] = v;
    
  }
  int h;
  
  for(h=0;h<=(top-bot);h++){
    hull.push_back(D[bot+h]);
  }
  for(ply_vertex* v:hull){
    cout<<v->getPos()<<"\n";
  }
  

  

 

  
}

}//end namespace polygon
}//end namespace masc
