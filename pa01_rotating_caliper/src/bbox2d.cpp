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
  v = v.normalize();
  // v & n are perpendicular
  n = Vector2d(-v[1],v[0]);
  // cout<<"v= "<<v<<"\n";
  // cout<<"n= "<<n<<"\n";
  vector<mathtool::Point2d> newHull = m_chull;
  Point2d origin = m_chull[0];
  Vector2d dif;
  for (int i =0;i<m_chull.size();i++){
    dif= Vector2d(m_chull[i]-origin);
    newHull[i] = Vector2d(v[0]*dif[0]+v[1]*dif[1],n[0]*dif[0]+n[1]*dif[1]);
  }

//Find vertex indices of extreme points 
  // e[0]: max v
  // e[1]: min v
  // e[2]: max n
  // e[3]: min n
  //initialize extreme point to the first vertex of convex hull
  //2. init extreme points e[4] using v & n, compute angles a[4]
  for (int i=0;i<4;i++){
    e[i] = 0;
  }
  for (int i =1; i<newHull.size();i++){
    
    if (newHull[i][0]>newHull[e[0]][0] || 
    (newHull[i][0] == newHull[e[0]][0] && newHull[i][1]>newHull[e[0]][1])){
      //maxX = m_chull[x][0];
      //index of vertex that have maximum value of v
      e[0]= i;
    }
    if(newHull[i][0]<newHull[e[1]][0]||
    (newHull[i][0] == newHull[e[1]][0] && newHull[i][1]<newHull[e[1]][1])){
      //minX = m_chull[x][0];
      //index of vertex that have minimum value of v
      e[1]= i;
    }
  }

  for (int j =1; j<newHull.size();j++){
    if (newHull[j][1]>newHull[e[2]][1]||
    (newHull[j][1] == newHull[e[2]][1] && newHull[j][0]>newHull[e[2]][0])){
      //maxY = m_chull[y][1];
      //index of vertex that have maximum value of n
      e[2] = j;
    }
    if(newHull[j][1]<newHull[e[3]][1]||
    (newHull[j][1] == newHull[e[3]][1] && newHull[j][0]<newHull[e[3]][0])){
      //minY = m_chull[y][1];
      //index of vertex that have minimum value of n
      e[3] = j;
    }
  }
  
  m_chull = newHull;
  //index of the vector that has the smallest angle
  minA = findAngles(e,a,v,n);
  
  //3. iteratively update extreme points
  for(int i=0;i<m_chull.size();i++)
  {
    //3.1 create a box from v,n,e[4]
    obb box = createOBB(e,v,n);
    // transform the corner to x-y coordinate
    for (int i =0;i<4;i++){
      double x = box.corners[i][0]*v[0] + box.corners[i][1] * n[0] + origin[0];
      double y = box.corners[i][0]*v[1] + box.corners[i][1]* n[1]+ origin[1];
      box.corners[i] = Vector2d(x,y);
    }
    //3.2 check if this box solve the problem (use problem.solved)
    problem.solved(box);

    //no need to update the m_chull again
    
    //3.3 update v,n,e[4],a[4]
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
  float minA;
  int i;
  Vector2d vec[4];
  for(int i=0;i<2;i++){
    // compute cosine theta to sort the angles
   
    if(i<=1){
      // e[0] & e[1] are the max/ min of v, compute the angle between the vector and n
      // take the absolute value of cosine theta (because there're two vectors n and -n)
      vec[i] = m_chull[e[i]+1] - m_chull[e[i]];
      a[i] = abs(vec[i][0]*n[0]+vec[i][1]*n[1])/vec[i].norm();
    }
    else{
      // e[1] & e[2] are the max/ min of n, compute the angle between the vector and v
      vec[i] =m_chull[e[i]+1] - m_chull[e[i]];
      a[i] = abs(vec[i][0]*v[0]+vec[i][1]*v[1])/vec[i].norm();
    }
  }
  // Vector2d v0 = m_chull[e[0]+1] - m_chull[e[0]];
  // a[0] = abs(v0[0]*n[0]+v0[1]*n[1])/v0.norm();
  // Vector2d v1 = m_chull[e[1]+1] - m_chull[e[1]];
  // a[1] = abs(v1[0]*n[0]+v1[1]*n[1])/v1.norm();
  // Vector2d v2 =m_chull[e[2]+1] - m_chull[e[2]];
  // a[2] = abs(v2[0]*v[0]+v2[1]*v[1])/v2.norm();
  // Vector2d v3 =m_chull[e[3]+1] - m_chull[e[3]];
  // a[3] = abs(v3[0]*v[0]+v3[1]*v[1])/v3.norm();
  minA = a[0];
  i=0;
  for(int k =0;k<4;k++){
    if(a[k]<minA){
      minA = a[k];
      i=k;
    }
  }

  return i;
}

obb bbox2d::createOBB(int e[4],const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
  obb box;
  
  //TODO: build a box from e, the extreme points, v and n
  // e[0]: max v
  // e[1]: min v
  // e[2]: max n
  // e[3]: min n
  //lower-left corner
  box.corners[0] = Point2d(m_chull[e[1]][0],m_chull[e[3]][1]);
  //lower-right
  box.corners[1] = Point2d(m_chull[e[0]][0],m_chull[e[3]][1]);
  //upper-right
  box.corners[2] = Point2d(m_chull[e[0]][0],m_chull[e[2]][1]);
  //upper-left
  box.corners[3] = Point2d(m_chull[e[1]][0],m_chull[e[2]][1]);
  box.height = m_chull[e[0]][0]-m_chull[e[1]][0];
  box.width = m_chull[e[2]][1]-m_chull[e[3]][1];

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
