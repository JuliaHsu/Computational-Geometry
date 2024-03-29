#pragma once

#include <list>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <set>
#include <fstream>
#include <iostream>
#include <limits>
#include <iomanip>
#include <numeric>
#include <iostream>

#undef check

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>



struct VorCell
{
	VorCell(){}

	VorCell(const VorCell& other)
	{
		site = other.site;
		coverage = other.coverage;
	}

	cv::Point site;
	std::list<cv::Point> coverage;
};

bool compareCell(const std::pair<float, cv::Point>& p1, const std::pair<float, cv::Point>& p2);

//centroidal Voronoi Tessellation (CVT)
struct CVT
{
public:

	CVT()
	{
		iteration_limit = 100;
		max_site_displacement = 1.01f;
		average_termination = false;
		debug = false;
	}

	void compute_weighted_cvt(cv::Mat &  img, std::vector<cv::Point2d> & pts);
	
	const std::vector<VorCell> & getCells() const
	{
		return this->cells;
	}

	int iteration_limit;       //max number of iterations when building cvf
	float max_site_displacement; //max tolerable site displacement in each iteration.
	bool average_termination;
	bool debug;

private:

	static std::vector<VorCell> cells;
	void vor_sub(cv::Mat & img);
	void vor(cv::Mat &  img); 

	//convert a color intensity to distance between 0~1
	inline float color2dist(cv::Mat &  img, cv::Point & p)
	{
		//note: 256 is used here instead of 255 to prevent 0 distance.
		//black =0; white = 255
		
		// cv::Mat dst;
		// cv::Point2f pt(p.y, p.x);
		// cv::getRectSubPix(img,cv::Size(1,1),pt,dst);
		// std::cout<<int(img.at<uchar>(p.x,p.y))<<std::endl;
		// std::cout<<int(dst.at<uchar>(0,0))<<std::endl;
		
		// return (256 - dst.at<uchar>(0,0))*1.0f / 256;
		
		return (256 - img.at<uchar>(p.x, p.y))*1.0f / 256;
	}

	//move the site to the center of its coverage
	//refer to paper sec 2.2: Loyd's method: move site to its centroid
	//check convergence
	inline float move_sites(cv::Mat &  img, VorCell & cell)
	{
		if (cell.coverage.empty()) std::cout << "! Error: cell.coverage " << cell.site << " size = " << cell.coverage.size() << std::endl;

		//Generate virtual high resolution image;
		
		cv::Size res(img.size().width, img.size().height);
		// cv::Mat resizedImg(res.width, res.height, cv::IMREAD_GRAYSCALE);
		// cv::resize(img, resizedImg, res, 0, 0, cv::INTER_LINEAR);
		// cv::Point2f pt(res.width/2, res.height/2);
		// cv::getRectSubPix(resizedImg,res,pt,resizedImg);
		//compute weighted average
		float total = 0;
		cv::Point2d new_pos(0, 0);
		//cell.coverage: the points in the cell c
		for (auto& c : cell.coverage)
		{
			float d = color2dist(img, c);
			//indtegral of density * position
			new_pos.x += d*c.x;
			new_pos.y += d*c.y;
			//integral of density
			total += d;
		}

		//normalize
		new_pos.x /= total;
		new_pos.y /= total;


		//update
		//dist: distance move from the old site to the new site
		float dist = fabs(new_pos.x - cell.site.x) + fabs(new_pos.y - cell.site.y); //manhattan dist
		cell.site = new_pos;

		//done
		
		return dist;
	}

	//move the sites to the centers of their coverages
	inline float move_sites(cv::Mat &  img)
	{
		float max_offset = 0;
		if (average_termination)
		{
			for (auto& cell : this->cells)
			{
				//cout << "coverage size=" << cvt.cells[607].coverage.size() << endl;
				float offset = move_sites(img, cell);
				max_offset += offset;
			}
			//average distance moved by all old sites
			max_offset /= this->cells.size();
		}
		else
		{
			for (auto& cell : this->cells)
			{
				//cout << "coverage size=" << cvt.cells[607].coverage.size() << endl;
				float offset = move_sites(img, cell);
				if (offset > max_offset)
					max_offset = offset;
			}
		}
		return max_offset;
	}


	
};
