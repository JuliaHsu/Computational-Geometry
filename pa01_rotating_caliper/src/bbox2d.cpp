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

double dot(Vector2d v, Vector2d dif){
  return v[0]*dif[0]+v[1]*dif[1];
}

obb bbox2d::build(bbox2d_problem & problem)
{
  mathtool::Vector2d v,n; //the calipers, v & n are perpendicular
  int e[4]; //vertex indices of extreme points
  float a[4]; //angles between the calipers and the polygon edges
//Find vertex indices of extreme points 
  // e[0]: max X
  // e[1]: min X
  // e[2]: max Y
  // e[3]: min Y
  for (int i=0;i<4;i++){
    e[i] = 0;
  }
  for (int x =1; x<m_chull.size();x++){
    
    if (m_chull[x][0]>m_chull[e[0]][0] || 
    (m_chull[x][0] == m_chull[e[0]][0] && m_chull[x][1]>m_chull[e[0]][1])){
      //maxX = m_chull[x][0];
      //index of vertex that have maximum value of x
      e[0]= x;
    }
    if(m_chull[x][0]<m_chull[e[1]][0]||
    (m_chull[x][0] == m_chull[e[1]][0] && m_chull[x][1]<m_chull[e[1]][1])){
      //minX = m_chull[x][0];
      //index of vertex that have minimum value of x
      e[1]= x;
    }
  }

  for (int y =1; y<m_chull.size();y++){
    if (m_chull[y][1]>m_chull[e[2]][1]||
    (m_chull[y][1] == m_chull[e[2]][1] && m_chull[y][0]>m_chull[e[2]][0])){
      //maxY = m_chull[y][1];
      //index of vertex that have maximum value of y
      e[2] = y;
    }
    if(m_chull[y][1]<m_chull[e[3]][1]||
    (m_chull[y][1] == m_chull[e[3]][1] && m_chull[y][0]<m_chull[e[3]][0])){
      //minY = m_chull[y][1];
      //index of vertex that have minimum value of y
      e[3] = y;
    }
  }
  

  //1. initialize v so it is parallel to an edge and then determine n
  
  // m_chull[e[0]]: the extreme point of convex hull with maximum value of x
  // (m_chull[e[0]+1],m_chull[e[0]]) : the edge that contains m_chull[e[0]] as a start point
   v = Vector2d(m_chull[e[0]+1]-m_chull[e[0]]);
   v = v.normalize();
   // v & n are perpendicular
   n = Vector2d(-v[1],v[0]);
  cout<<"v= "<<v<<"\n";
  cout<<"n= "<<n<<"\n";
  cout<<"extreme points\n";
  for(int i=0;i<4;i++){
    cout<<m_chull[e[i]]<<"\n";
  }
  cout<<"---------------\n";
  //2. init extreme points e[4] using v & n, compute angles a[4]
  Point2d origin = m_chull[e[0]];
  Vector2d newEp[4];
  for(int i=0;i<4;i++){
    Vector2d dif= Vector2d(m_chull[e[i]]-origin);
    newEp[i] = Vector2d(dot(v,dif),dot(n,dif));
   

    findAngles(e,a,v,n);
  }
  for(int i =0;i<4;i++){
    m_chull[e[i]] =  newEp[i];
    cout<<m_chull[e[i]]<<"\n";
  }

  //3. iteratively update extreme points
  for(int i=0;i<m_chull.size();i++)
  {
    //3.1 create a box from v,n,e[4]
    //3.2 check if this box solve the problem (use problem.solved)
    //no need to update the m_chull again
    if(i==e[0]|| i==e[1]||i==e[2] ||i==e[3])
      continue;
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

  return 0;
}

obb bbox2d::createOBB(int e[4],const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
  obb box;

  //TODO: build a box from e, the extreme points, v and n

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
