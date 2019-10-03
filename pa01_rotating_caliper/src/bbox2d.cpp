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
}


obb bbox2d::build(bbox2d_problem & problem)
{
  
  mathtool::Vector2d v,n; //the calipers, v & n are perpendicular
  int e[4]; //vertex indices of extreme points
  float a[4]; //angles between the calipers and the polygon edges
  int minA =0;
  bool processed [m_chull.size()-1];
  for(int i = 0;i<m_chull.size()-1;i++){
    processed[i]=false;
  }
  //1. initialize v so it is parallel to an edge and then determine n
  //e0
  v = Vector2d(m_chull[1]-m_chull[0]);
  // v & n are perpendicular
  n = Vector2d(-v[1],v[0]);
  processed[0] = true;
  vector<mathtool::Point2d> newHull = m_chull;
  Point2d origin = m_chull[1];
  Vector2d diff;
  Vector2d vec[4];
  
  for (int i =0;i<m_chull.size();i++){
    diff= Vector2d(m_chull[i]-origin);
    newHull[i] = Vector2d(v*diff,n*diff);
    //cout<<newHull[i]<<"\n";
  }
  //Find vertex indices of extreme points 
  //initialize extreme point to the first vertex of convex hull
  //2. init extreme points e[4] using v & n, compute angles a[4]
  for (int i=0;i<4;i++){
    e[i] = 1;
  }
  for (int i =1; i<newHull.size()-1;i++){
    
    if (newHull[i][0]>newHull[e[1]][0] || 
    (newHull[i][0] == newHull[e[1]][0] && newHull[i][1]>newHull[e[1]][1])){
      //index of vertex that have maximum value of v, or same maximum of v with bigger n
      //upper-right
      e[1]= i;
    }
    if(newHull[i][1]>newHull[e[2]][1]||
    (newHull[i][1] == newHull[e[2]][1] && newHull[i][0]<newHull[e[2]][0])){
      //index of vertex that have maximum value of n, or same n but smaller v
      //upper-left
      e[2]= i;
    }
    if(newHull[i][0]<newHull[e[3]][0]||
    (newHull[i][0] == newHull[e[3]][0] && newHull[i][1]<newHull[e[3]][1])){
      //index of vertex that have minimum value of v, or same v with smaller n
      //lower-left
      e[3]= i;
    }
  }

  //compute angles
  for(int i =0;i<4;i++){
    if(e[i] == m_chull.size()-1){
      vec[i] = m_chull[1] - m_chull[0];
      
    }
    else{
      vec[i] = m_chull[e[i]+1] - m_chull[e[i]];
      
    }
  
    if(i%2 == 1){
      //a[i] = vec[i].angleTo(n);
      //cosine^2 
      a[i] = pow(vec[i]*n/(vec[i].norm()*n.norm()),2);
      //cout<<"a: "<<a[i]<<", ";
    }
    else{
      
      // a[i] =vec[i].angleTo(v);
      // cosine^2
      a[i] = pow(vec[i]*v/(vec[i].norm()*v.norm()),2);
       //cout<<"a: "<<a[i]<<", ";
    }
  }
  
    
 
  //index of the vector that has the smallest angle
  //3. iteratively update extreme points
  Vector2d Vn,Nn;
 
  for(int i=0;i<m_chull.size();i++)
  {
    //3.1 create a box from v,n,e[4]
    obb box = createOBB(e,v,n);
    Vn = v.normalize();
    Nn = n.normalize();
    //3.2 check if this box solve the problem (use problem.solved)
    problem.solved(box);

    //3.3 update v,n,e[4],a[4]
    minA = findAngles(e,a,v,n);
    if(processed[e[minA]]){
      break;
    }
    v =m_chull[e[minA]+1]-m_chull[e[minA]];
    //v = v.normalize();
    n = Vector2d(-v[1],v[0]);
    origin = m_chull[e[minA]+1];
    processed[e[minA]] = true;
    //update extreme points
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
    
    
    //rotate the extreme points so that e[minA] occurs first to maintain the order of the extreme points as begining
    int tmpE[4];
    for(int i=0;i<4;i++){
      
      tmpE[i] = e[(minA+i)%4];
    }
    copy(begin(tmpE), end(tmpE), begin(e));

    Point2d eps[4];
    Point2d eps2[4];
    Vn = v.normalize();
    Nn = n.normalize();

    for(int i=0;i<4;i++){
      diff = Vector2d(m_chull[e[i]]-origin);
      eps[i] = Vector2d(v*diff,n*diff);
      //cout<<eps[i];
    }
    for(int i=0;i<4;i++){
      diff = Vector2d(m_chull[e[i]+1]-origin);
      eps2[i] = Vector2d(v*diff,n*diff);
      //cout<<eps2[i];
    }
 
    //update angles
    for(int i =0;i<4;i++){
      vec[i] = m_chull[e[i]+1]-m_chull[e[i]];
      if(i%2 == 1){
        // a[i] = vec[i].angleTo(n);
        //cosine^2
        a[i] = pow(vec[i]*n/(vec[i].norm()*n.norm()),2);
        //cout<<"a: "<<a[i]<<", ";
      }
      else{
        
        //a[i] = vec[i].angleTo(v);
        // cosine^2
        a[i] = pow(vec[i]*v/(vec[i].norm()*v.norm()),2);
        //cout<<"a: "<<a[i]<<", ";
      }
    }
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
  float maxCos;
  int i=0;
  //maximum cos == minimum angle
  maxCos = a[0];
  for(int k =1;k<4;k++){
    if((a[k]>maxCos)){
      maxCos = a[k];
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
  origin = m_chull[e[0]];
  Vector2d Vn= v.normalize();
  Vector2d Nn = n.normalize();

  for(int i=0;i<4;i++){
    diff = Vector2d(m_chull[e[i]]-origin);
    eps[i] = Point2d(diff*Vn,diff*Nn);
  }

  //TODO: build a box from e, the extreme points, v and n
  int maxV, minV, maxN, minN;
  maxV =0;
  minV =0;

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

  // // transform the corners to x-y coordinate
  for(int i =0;i<4;i++){
    double x = box.corners[i][0]*Vn[0] + box .corners[i][1] * Nn[0]+origin[0];
    double y = box.corners[i][0]* Vn[1] + box .corners[i][1]* Nn[1]+origin[1];
    box .corners[i] = Point2d(x,y);
  }
  box.height = (box.corners[0] - box.corners[1]).norm();
  box.width = (box.corners[2] - box.corners[1]).norm();
  // cout<<"H= "<<box.height<<" W= "<<box.width;

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