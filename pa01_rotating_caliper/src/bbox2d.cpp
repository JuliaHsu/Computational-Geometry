#include "bbox2d.h"
#include "chull.h"
#include "simple_svg_1.0.0.hpp"

namespace masc {
namespace polygon {

//initialize with a given polygon
bbox2d::bbox2d(const c_polygon & poly)
{
  const c_ply& ply=poly.front();
  list<ply_vertex*> hull;
  hull2d(ply.getHead(), ply.getHead()->getPre(), hull);

  for(ply_vertex* v:hull)
  {

    auto pos=v->getPos();
    this->m_chull.push_back(pos);
  }//end for
 
//  for(int i=0;i<m_chull.size();i++){
//    cout<<m_chull[i]<<"\n";
//  }
  


  
  

  

}



obb bbox2d::build(bbox2d_problem & problem)
{
  
  mathtool::Vector2d v,n; //the calipers, v & n are perpendicular
  int e[4]; //vertex indices of extreme points
  float a[4]; //angles between the calipers and the polygon edges
  int minA =0;
  //1. initialize v so it is parallel to an edge and then determine n
  //e0
  v = Vector2d(m_chull[1]-m_chull[0]);
  //v = v*10.0;
  //v = v.normalize();
  // v & n are perpendicular
  n = Vector2d(-v[1],v[0]);
  cout<<"v= "<<v<<"\n";
  cout<<"n= "<<n<<"\n";
  vector<mathtool::Point2d> newHull = m_chull;
  Point2d origin = m_chull[1];
  Vector2d dif;
  Vector2d vec[4];
  bool visited[m_chull.size()-1];

  for(int i =0;i<m_chull.size()-1;i++){
    visited[i] = false;
  }
  visited[0] =true;
  // cout<<"m_chull:\n";
  // for(int i =0;i<m_chull.size();i++){
  //   cout<<m_chull[i]<<"\n";
  // }
  //cout<<"new hull: \n";
  // cout<<"17: "<<m_chull[17]<<", 18: "<<m_chull[18]<<", 19: "<<m_chull[19];
  for (int i =0;i<m_chull.size();i++){
    dif= Vector2d(m_chull[i]-origin);
    newHull[i] = Vector2d(v[0]*dif[0]+v[1]*dif[1],n[0]*dif[0]+n[1]*dif[1]);
    //cout<<newHull[i]<<"\n";
  }
// cout<<"new:\n";
// cout<<"17: "<<newHull[17]<<", 18: "<<newHull[18]<<", 19: "<<newHull[19];
//Find vertex indices of extreme points 
  //initialize extreme point to the first vertex of convex hull
  //2. init extreme points e[4] using v & n, compute angles a[4]
  for (int i=0;i<4;i++){
    e[i] = 1;
  }
  for (int i =0; i<newHull.size()-1;i++){
    
    if (newHull[i][0]>newHull[e[1]][0] || 
    (newHull[i][0] == newHull[e[1]][0] && newHull[i][1]>newHull[e[1]][1])){
      //maxX = m_chull[x][0];
      //index of vertex that have maximum value of v
      e[1]= i;
    }
    if(newHull[i][1]>newHull[e[2]][1]||
    (newHull[i][1] == newHull[e[2]][1] && newHull[i][0]<newHull[e[2]][0])){
      //minX = m_chull[x][0];
      //index of vertex that have minimum value of v
      e[2]= i;
    }
    if(newHull[i][0]<newHull[e[3]][0]||
    (newHull[i][0] == newHull[e[3]][0] && newHull[i][1]<newHull[e[3]][1])){
      //minX = m_chull[x][0];
      //index of vertex that have minimum value of v
      e[3]= i;
    }
  }

  cout<<"extreme points:\n";
  for(int i=0;i<4;i++){
    cout<<"e[i]= "<<e[i]<<", point:"<<m_chull[e[i]]<<"\n";
  }
  //cout<<"e[18]: "<<newHull[18]<<"\n";
  Vector2d Eperp;
  Vector2d zero = Vector2d(0,0);


  //compute angles
  for(int i =0;i<4;i++){
    if(e[i] == m_chull.size()-1){
      vec[i] = m_chull[1] - m_chull[0];
      //cout<<e[1]<<" - "<<e[0]<<"\n";
    }
    else{
      vec[i] = m_chull[e[i]+1] - m_chull[e[i]];
      //cout<<e[i]+1<< "- "<<e[i]<<"\n";
    }
    
    //cout<<"vec: "<<vec[i]<<", ";

  
    Eperp = Vector2d(-vec[i][1],vec[i][0]);
    if(i%2 == 1){
      a[i] = ((n*Eperp) * (n*Eperp))/ (Eperp.norm()* Eperp.norm());
      //a[i] = abs(vec[i][0]*n[0]+vec[i][1]*n[1])/vec[i].norm();
      cout<<"a: "<<a[i]<<", ";
    }
    else{
      // cosine theta
      // |v|^2 sin^2
       a[i] = ((v*Eperp) * (v*Eperp))/ (Eperp.norm()* Eperp.norm());
       //a[i] = abs(vec[i][0]*v[0]+vec[i][1]*v[1])/vec[i].norm();
       cout<<"a: "<<a[i]<<", ";
    }
  }
  
    
 
  //index of the vector that has the smallest angle
  //3. iteratively update extreme points
  int idxA, tmp;
  float tmpA;
  Vector2d Vn,Nn;
 
  for(int i=0;i<1;i++)
  {
  
  //   //3.1 create a box from v,n,e[4]
    obb box = createOBB(e,v,n);
    
   
    Vn = v.normalize();
    Nn = n.normalize();
    // transform the corner to x-y coordinate
    for (int i =0;i<4;i++){
      double x =  box .corners[i][0]*Vn[0] + box .corners[i][1] * Nn[0] + origin[0];
      double y = box .corners[i][0]* Vn[1] + box .corners[i][1]* Nn[1]+ origin[1];
      box .corners[i] = Vector2d(x,y);
    }
  //   //3.2 check if this box solve the problem (use problem.solved)
    cout<< "finished building a box!\n";
    cout<<"---------------\n";
    problem.solved(box);
    
  //3.3 update v,n,e[4],a[4]
  //update e[4]
  // for(int i=0;i<4;i++){
  //   cout<<"a = "<<a[i]<<", e[i] =  "<< e[i]<<"\n";
  // }
    minA = findAngles(e,a,v,n);
    //cout<<"dist: "<<(m_chull[19]-m_chull[18]).norm();
    //cout<<"minA: "<<a[minA]<<"e: "<<e[minA]<<"\n";
    // if((m_chull[e[minA]+2]-m_chull[e[minA]+1]).norm()<=0.09){
    //   a[minA] = 1000;
    //   minA = findAngles(e,a,v,n);
    // }
   
    //check duplicate vertex
    // for(int i=0;i<4;i++){
    //   for(int j=i+1;j<4;j++){
    //     if(e[i] == e[j] && minA!=i){
    //       // make a[i] be the maximum, so it will be skipped
    //       a[i]=100000;
    //       minA = findAngles(e,a,v,n);
    //     }
    //   }
    // }
    

    //cout<<"minA = "<<m_chull[e[minA]+1]-m_chull[e[minA]]<<"\n";
    if(visited[e[minA]]){
      break;
    }
    v =m_chull[e[minA]+1]-m_chull[e[minA]];
    //v = v.normalize();
    // v = v*10;
    n = Vector2d(-v[1],v[0]);
    visited[e[minA]] = true;
    cout<<"new v: "<<  e[minA]+1<<" - "<<e[minA] <<" = "<<v<<"\n";
    cout<< "new n: "<<n<<"\n";
    origin = m_chull[e[minA]+1];
    cout<<"origin index= "<<e[minA]+1<<"\n";
    //cout<<"m_chull[e[minA]]: "<<m_chull[e[minA]] <<"\n";
    cout<<"old ep index:\n";
    for(int i=0;i<4;i++){
      cout<<e[i]<<", "<<m_chull[e[i]]<<"\n";
    }
    cout<<"\n";
    
    Vector2d tmp;
    Vector2d tmp2;
    //update extreme points
    //check duplicate vertex:
    bool isDuplicate= false;
    int dupMin;
    
    for(int i=0;i<4;i++){
      if(e[minA] == e[i] && minA!=i){
        isDuplicate= true;
        if(minA> i){
          minA =i;
        }
        cout<<"minA: "<<minA<<"\n";
        dupMin = i;
        cout<<"dup min: "<<dupMin<<"\n";
      }
    }
    //update extreme points
    if(isDuplicate){
      cout<< "duplicate\n";
      if(e[dupMin]==m_chull.size()-1){
        e[dupMin]= 1;  
      }
      else if(e[dupMin]==m_chull.size()-2){
        e[dupMin] = 0;
      }
      else{
        e[dupMin] = e[dupMin]+1;
      }
      cout<<e[dupMin];
    }
    else{
      cout<<"no duplicate\n";
      for(int i=0;i<4;i++){
        if(a[minA] == a[i]){
          if(e[i]==m_chull.size()-1){
            e[i]= 1;  
          }
          else if(e[i]==m_chull.size()-2){
            e[i] = 0;
          }
          else{
            e[i] = e[i]+1;
          }
        }
      }
    }
    cout<<"before rotation:\n";
    for(int i=0;i<4;i++){
      cout<<e[i]<<", ";
    }
    cout<<"\n";
    // for(int i=0;i<4;i++){
    //   if(a[minA] == a[i]){
    //     //update extreme points that have the same minimum angle
    //     if(!isDuplicate){
    //       if(e[i]==m_chull.size()-1){
    //         e[i]= 1;  
    //       }
    //       else if(e[i]==m_chull.size()-2){
    //         e[i] = 0;
    //       }
    //       else{
    //         e[i] = e[i]+1;
    //       }
    //     }
    //     else{
    //       cout<<"update "<<e[dupMin]<<"\n";
    //       //e[minA]= e[minA];
    //       if(dupMin == i ){
    //         //cout<<"update "<<e[dupMin]<<"\n";
    //         if(e[dupMin]==m_chull.size()-1){
    //           e[dupMin]= 1;  
    //         }
    //         else if(e[i]==m_chull.size()-2){
    //           e[dupMin] = 0;
    //         }
    //         else{
    //           e[dupMin] = e[dupMin]+1;
    //         }
    //       }
          
    //     }
        

    //   }
    //   cout<<e[i]<<", ";
    // }

    // for(int i=0;i<4;i++){
    //   if(e[minA] == e[i] && minA!=i){
    //     //minA must < i (because the search is in order), only update e[i]; no change of e[minA]
    //     if(e[i]==m_chull.size()-1){
    //       e[i] = 1;
    //     }
    //     else{
    //       e[i] = e[i]+1;
    //     }
    //   }
    //   //no duplicate vertex or there is a duplicate vertex, 
    //   //but the angle is not the minimum
    //   else{
    //     cout<<"mini e: "<<e[minA];
    //     if(e[minA]==m_chull.size()-1){
    //       e[minA]= 1;  
    //     }
    //     else{
    //       e[minA] = e[minA]+1;
    //     }
    //   }
    // }
    // for(int i=0;i<4;i++){
    //   if(a[minA] == a[i] && i!=minA){
    //     //update extreme points that have the same minimum angle, 
    //     //but different with e[minA]
    //     if(e[minA]==m_chull.size()-1){
    //       e[i]= 1;  
    //     }
    //     else{
    //       e[i] = e[i]+1;
    //     }

    //   }
    // }
    //rotate the extreme points so that e[minA] occurs first
    int tmpE[4];
    float tmpA[4];
    for(int i=0;i<4;i++){
      if(isDuplicate){
      
        tmpE[i] = e[(dupMin+i)%4];
        tmpA[i] = a[(dupMin+i)%4];
       
      }
      else{
        tmpE[i] = e[(minA+i)%4];
        tmpA[i] = a[(minA+i)%4];
        
      }
      
    }
    copy(begin(tmpE), end(tmpE), begin(e));
    cout<<"rotate array e:\n";
    for(int i=0;i<4;i++){
      cout<<e[i]<<", ";
    }

    
  
      

    Point2d eps[4];
    Point2d eps2[4];
    Vector2d diff;
    
    Vn = v.normalize();
    Nn = n.normalize();
    // // cout<<"new extreme point:\n";

    for(int i=0;i<4;i++){
      diff = Vector2d(m_chull[e[i]]-origin);
      eps[i] = Vector2d(Vn*diff,Nn*diff);
      //cout<<eps[i];
    }
    for(int i=0;i<4;i++){
      diff = Vector2d(m_chull[e[i]+1]-origin);
      eps2[i] = Vector2d(Vn*diff,Nn*diff);
      //cout<<eps2[i];
    }
    obb box2 = createOBB(e,v,n);

    double minV, maxV, minN,maxN;
    minV = box2 .corners[0][0] ;
    maxV = box2 .corners[2][0];
    minN = box2 .corners[0][1] ;
    maxN = box2 .corners[2][1] ;
    cout<<"minV: "<<minV<<"\n";
    cout<<"maxV: "<<maxV<<"\n";
    cout<<"minN: "<<minN<<"\n";
    cout<<"maxN:  "<<maxN<<"\n";

    cout<<"new ep index:\n";
    for(int i=0;i<4;i++){
      cout<<e[i]<<", "<<eps[i]<<"\n";
    } 
 
    //update angles
    for(int i =0;i<4;i++){
      vec[i] = eps2[i]-eps[i];
      Eperp = Vector2d(-vec[i][1],vec[i][0]);

      if(i%2 == 1){
        a[i] = ((n*Eperp) * (n*Eperp))/ (Eperp.norm()* Eperp.norm());
        //a[i] = abs(vec[i][0]*n[0]+vec[i][1]*n[1])/vec[i].norm();
        cout<<"a: "<<a[i]<<", ";
      }
      else{
        // cosine theta
        // |v|^2 sin^2
        a[i] = ((v*Eperp) * (v*Eperp))/ (Eperp.norm()* Eperp.norm());
        //a[i] = abs(vec[i][0]*v[0]+vec[i][1]*v[1])/vec[i].norm();
        cout<<"a: "<<a[i]<<", ";
      }
      
      // if(eps[i][0] == minV || eps[i][0]== maxV){
      //   //the extreme point is on v
      //   a[i] = ((n*Eperp) * (n*Eperp))/ (Eperp.norm()* Eperp.norm());
      //   //a[i] = abs(vec[i]*n)/vec[i].norm();
      //   cout<<"a: "<<a[i]<<", ";
      //   cout<<"the extreme point is on v\n";

      // }
      
      // else if(eps[i][1]==minN || eps[i][1]== maxN){
      //   //a[i] = abs(vec[i]*v)/vec[i].norm();
      //   a[i] = ((v*Eperp) * (v*Eperp))/ (Eperp.norm()* Eperp.norm());
      //   cout<<"a: "<<a[i]<<", ";
      //   cout<<"the extreme point is on n\n";

      // }
      // else{
      //  //a[i] = ((n*Eperp) * (n*Eperp))/ (Eperp.norm()* Eperp.norm());
      //   //a[i]=0;
      //   cout<<"a[i]: "<<a[i];
      //   cout<<" else!\n";
      // }

    }
    bool isInE = false;
    for(int i =0;i<4;i++){
      if(origin == m_chull[e[i]]){
        isInE = true;
      }
    }
   // cout<<"\nisInE: "<< isInE<<"\n";

  }

  return problem.getSolution(); //done
}

//TODO:
//compute "angles" or some values that can be used to sort angles between
//the caliper and the edges
//
// e: 4 extreme points on the convex hull
// a: the angles to be computed between the calipers and the edges incident to the extreme points
// u, v: the calipers
//
// return a value i=0~4, where a[i] is the smallest
int bbox2d::findAngles
(int e[4], float a[4], const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
  float minSin;
  int i=0;
  // while(a[i]==0 && i<4){
  //   i++;
  // }
  // minSin = a[i--];
  minSin = a[0];
  for(int k =1;k<4;k++){
    if((a[k]<minSin)){
      minSin = a[k];
      i=k;
    }
  }


  return i;
}

obb bbox2d::createOBB(int e[4],const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
  obb box;
  Vector2d diff;
  Point2d eps[4];
  Point2d origin ;

  Vector2d tmp;
  //cout<<"extreme point in box\n";
  
  for(int i =0;i<4;i++){
    //cout<<"m_chull[e[i]]-m_chull[e[i]-1] = "<< m_chull[e[i]]-m_chull[e[i]-1]<<"\n";
    tmp= Vector2d(m_chull[e[i]]-m_chull[e[i]-1]);
    // cout<<"v= "<<v<<"\n";
    // cout<< "tmp = "<< m_chull[e[i]]-m_chull[e[i]-1]<<"\n";
    if(tmp ==v){
      origin = m_chull[e[i]];
      cout<<"origin: "<<origin<<"\n";
      
    }
    
  }
  Vector2d Vn= v.normalize();
  Vector2d Nn = n.normalize();
  //cout<<"\nnew eps:\n";
  for(int i=0;i<4;i++){
    diff = Vector2d(m_chull[e[i]]-origin);

    eps[i] = Point2d(Vn[0]*diff[0]+Vn[1]*diff[1],Nn[0]*diff[0]+Nn[1]*diff[1]);
    //cout<<eps[i]<<"\n";
    cout<<"diff: "<<diff<<"\n";
  }

  

  //TODO: build a box from e, the extreme points, v and n
  // e[0]: max v
  // e[1]: min v
  // e[2]: max n
  // e[3]: min n
  //lower-left corner
  int maxV, minV, maxN, minN;
  maxV =0;
  minV =0;
  // for(int i =1;i<4;i++){
  //   if(m_chull[e[i]][0]>m_chull[e[maxV]][0]){
  //     maxV = i;
  //   }
  //   else if (m_chull[e[i]][0]<m_chull[e[minV]][0]){
  //     minV = i;
  //   }
  // }

  for(int i =1;i<4;i++){
    if(eps[i][0]>eps[maxV][0]){
      maxV = i;
    }
    else if(eps[i][0]<eps[minV][0]){
      minV = i;
    }
  }

  maxN =0;
  minN =0;

  // for(int i =1;i<4;i++){
  //   if(m_chull[e[i]][1]>m_chull[e[maxN]][1]){
  //     maxN = i;
  //   }
  //   else if (m_chull[e[i]][1]<m_chull[e[minN]][0]){
  //     minN = i;
  //   }
  // }

  for(int i =1;i<4;i++){
    if(eps[i][1]>eps[maxN][1]){
      maxN = i;
    }
    else if(eps[i][1]<eps[minN][1]){
      minN = i;
    }
  }
  

 
  
  
  
  //lower-left
  box.corners[0] = Point2d(eps[minV][0],eps[minN][1]);
  //lower-right
  box.corners[1] = Point2d(eps[maxV][0],eps[minN][1]);
  //upper-right
  box.corners[2] = Point2d(eps[maxV][0],eps[maxN][1]);
  //upper-left
  box.corners[3] = Point2d(eps[minV][0],eps[maxN][1]);
  box.height = (eps[maxV][0]-eps[minV][0]);
  box.width = (eps[maxN][1]-eps[minN][1]);
  cout<<"H= "<<box.height<<" W= "<<box.width;
  cout<<"\n";


  return box;
}

//
// DO NOT CHANGE ANYTHING BELOW
//
// 3 different types of bounding box
//


//the problem of finding the minimum area bounding box
bool min_area_bbox::solved(const obb& box)
{
  float area=box.width*box.height;
  if(area<m_min_area){
    m_min_area=area;
    this->m_solution=box;
  }
  return false; //always return false so search continues;
}

//the problem of finding the minimum boundary bounding box
bool min_perimeter_bbox::solved(const obb& box)
{
  float peri=box.width+box.height;
  if(peri<m_min_peri){
    m_min_peri=peri;
    this->m_solution=box;
  }
  return false; //always return false so search continues;
}

//the problem of finding a bounding box that can fit into another box
bool contained_bbox::solved(const obb& box)
{
  if( (box.width<=m_width && box.height <= m_height) ||
      (box.height<=m_width && box.width <= m_height) )
  {
    this->m_solution=box;
    return true;
  }

  return false;
}

//stream out the box
std::ostream & operator<<(std::ostream& out, const obb& box)
{
  out<<"[w="<<box.width<<", h="<<box.height<<"], ("
     <<box.corners[0]<<") ,("<<box.corners[1]<<"), ("
     <<box.corners[2]<<") ,("<<box.corners[3]<<")";
  return out;
}

//
//SVG related methods below. DO NOT change!
//
//function for saving svg file
//
void ply2ply(const masc::polygon::c_ply& ply, svg::Polygon& poly)
{
    auto v=ply.getHead();
    do{
      auto & pos=v->getPos();
      poly << svg::Point(pos[0], pos[1]);
      v=v->getNext();
    }
    while(v!=ply.getHead());
    poly.endBoundary();
}

void box2ply(const masc::polygon::obb& box, svg::Polygon& poly)
{
    poly << svg::Point(box.corners[0][0], box.corners[0][1]);
    poly << svg::Point(box.corners[1][0], box.corners[1][1]);
    poly << svg::Point(box.corners[2][0], box.corners[2][1]);
    poly << svg::Point(box.corners[3][0], box.corners[3][1]);
    poly.endBoundary();
}

void saveSVG(string svg_filename, masc::polygon::c_ply& ply, const masc::polygon::obb& box)
{
    auto R=ply.getRadius();
    auto center=ply.getCenter();

    svg::Dimensions dimensions(R*2.5, R*2.5);
    svg::Document doc(svg_filename, svg::Layout(dimensions, svg::Layout::BottomLeft, 1, svg::Point(-center[0]+R*1.25, -center[1]+R*1.25)));

    //------------------------------------------------------------------
    //draw the external boundary
    svg::Polygon box_bd(svg::Fill(svg::Color::Yellow), svg::Stroke(0.5, svg::Color::Black));
    box2ply(box, box_bd);
    doc << box_bd;
    svg::Polygon poly_bd(svg::Fill(svg::Color::Silver), svg::Stroke(0.5, svg::Color::Black));
    ply2ply(ply, poly_bd);
    doc << poly_bd;

    doc.save();
    cout << "- Saved " << svg_filename << endl;
}

}}//end namespaces