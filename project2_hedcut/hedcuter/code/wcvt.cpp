#include "wcvt.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

std::vector<VorCell> CVT::cells;


bool compareCell(const std::pair<float, cv::Point>& p1, const std::pair<float, cv::Point>& p2)
{
	//p1.first: distance
	//p2.second: pix, position
	//if their distances are equaled
	if (p1.first == p2.first)
	{
		if (p1.second.x == p2.second.x) return p1.second.y > p2.second.y;
		return p1.second.x > p2.second.x;
	}
	// distances are not equaled
	//true if p1.first > p2.first --> p1's distances > p2's distances
	return p1.first > p2.first;
}

//build the VOR once
void CVT::vor(cv::Mat &  img)
{
	int w =0;
	int h =0;
	//Generate virtual high resolution image
	w = img.size().width ;
	h = img.size().height;

	cv::Size res(w, h);
	cv::Mat resizedImg(res.width, res.height, cv::IMREAD_GRAYSCALE);
	
	cv::resize(img, resizedImg, res, 0, 0, cv::INTER_LINEAR);
	// cv::Point2f pt(res.width/2, res.height/2);
	// cv::getRectSubPix(resizedImg,res,pt,resizedImg);
	
	// cv::resize(img, resizedImg, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
	//dist: a matrix that stores the distance value of each pixel
	cv::Mat dist(res, CV_32F, cv::Scalar::all(FLT_MAX));
	//root: a matrix that stores the site id for each pixel
	cv::Mat root(res, CV_16U, cv::Scalar::all(USHRT_MAX));
	cv::Mat visited(res, CV_8U, cv::Scalar::all(0));

	//init
	std::vector< std::pair<float, cv::Point> > open;
	ushort site_id = 0;
	//for every cell
	for (auto& c : this->cells)
	{
		if (debug)
		{
			if (c.site.x<0 || c.site.x>res.height)
				std::cout << "! Warning: c.site.x=" << c.site.x << std::endl;
				

			if (c.site.y<0 || c.site.y>res.width)
				std::cout << "! Warning: c.site.y=" << c.site.y << std::endl;
		}

		// cv::Mat dst;
		// cv::Point2f pt(pix.x, pix.y);
		// cv::getRectSubPix(resizedImg,cv::Size(1,1),pt,dst);
		// cv::Point p(0,0);
		// float d = color2dist(dst, p);
		
		cv::Point pix((int)c.site.x, (int)c.site.y);
		// cv::Point2f pt(pix.x, pix.y);
		// cv::getRectSubPix(resizedImg,res,pt,resizedImg);
		// cv::Point p(0,0);
		// float d = color2dist(resizedImg, p);
		
		float d = color2dist(resizedImg, pix);

		//stores distances of site
		dist.at<float>(pix.x, pix.y) = d;
		//root stores the site ids
		root.at<ushort>(pix.x, pix.y) = site_id++;
		open.push_back(std::make_pair(d, pix));
		c.coverage.clear();
	}
	//build the max heap with the distance (intensity)
	std::make_heap(open.begin(), open.end(), compareCell);

	//propagate
	while (open.empty() == false)
	{
		//The element with the highest value is moved to (end-1)
		std::pop_heap(open.begin(), open.end(), compareCell);
		// max site (the site that has the highest intensity)
		auto cell = open.back();
		//position of cell
		auto& cpos = cell.second;
		open.pop_back();

		//check if the distance from this cell is already updated
		if (cell.first > dist.at<float>(cpos.x, cpos.y)) continue;
		if (visited.at<uchar>(cpos.x, cpos.y) > 0) continue; //visited
		visited.at<uchar>(cpos.x, cpos.y) = 1;
		// cv::Mat dst;
		// cv::Point2f pt(cpos.x, cpos.y);
		
		// cv::imshow("dst", dst);

		//check the neighbors
		for (int dx =-1; dx <= 1; dx++) //x is row
		{
			//cpos is the site
			
			int x = cpos.x + dx;
			double xf = cpos.x + (double)dx/2;
			if (x < 0 || x >= res.height) continue;
			for (int dy = -1; dy <= 1; dy++) //y is column
			{
				if (dx == 0 && dy == 0) continue; //itself...

				int y = cpos.y + dy;
				double yf = cpos.y + (double)dy/2;
				if (y < 0 || y >= res.width) continue;
				//neighbor(x,y)
                cv::Point pt_tmp(x, y);
				// cv::Point2f tmp (xf,yf);
				// current point's (cpos) intensity + neighbor's intensity
				// minimum-energy configuration
				//we want to minimize the current point's (cpos) intensity + neighbor's intensity (refer to the paper sec 2.1)
				float newd = dist.at<float>(cpos.x, cpos.y) + color2dist(resizedImg, pt_tmp);
				
				// float newd = dist.at<float>(cpos.x, cpos.y) + color2dist(resizedImg, pt_tmp);
				//distance of neighbor (** initial distance of neighbor = FLT_MAX)
				float oldd = dist.at<float>(x, y);
				

				if (newd < oldd)
				{
					//set newd (the minimum distance so far) to the distance of neighbor (x,y)
					dist.at<float>(x, y)=newd;
					//set the site id of cpos to its neighbor
					root.at<ushort>(x, y) = root.at<ushort>(cpos.x, cpos.y);
					open.push_back(std::make_pair(newd, cv::Point(x, y)));
					std::push_heap(open.begin(), open.end(), compareCell);
				}
			}//end for dy
		}//end for dx
	}//end while

	//collect cells
	//put the points to the corresponding vor reigions
	for (int x = 0; x < res.height; x++)
	{
		for (int y = 0; y < res.width; y++)
		{
			//site id
			ushort rootid = root.at<ushort>(x, y);
			//points that have the same rootid belong to the same cell
			this->cells[rootid].coverage.push_back(cv::Point(x,y));
		}//end y
	}//end x

	//remove empty cells...
	int cvt_size = this->cells.size();
	for (int i = 0; i < cvt_size; i++)
	{
		if (this->cells[i].coverage.empty())
		{
			this->cells[i] = this->cells.back();
			this->cells.pop_back();
			i--;
			cvt_size--;
		}
	}//end for i

	if (debug)
	{
		//this shows the progress...
		double min;
		double max;
		cv::minMaxIdx(dist, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(dist, adjMap, 255 / max);
		//cv::applyColorMap(adjMap, adjMap, cv::COLORMAP_JET);

		for (auto& c : this->cells)
		{
			cv::circle(adjMap, cv::Point( c.site.y, c.site.x ), 2, CV_RGB(0, 0, 255), -1);
		}

		cv::imshow("CVT", adjMap);
		cv::waitKey(5);
	}
}

void CVT::vor_sub(cv::Mat & img){
	//Generate virtual high resolution image
	cv::Size res(img.size().width, img.size().height);
	// cv::Mat resizedImg(res.width, img.size().height, cv::IMREAD_GRAYSCALE);
	//dist: a matrix that stores the distance value of each pixel
	cv::Mat dist(res, CV_32F, cv::Scalar::all(FLT_MAX));
	//root: a matrix that stores the site id for each pixel
	cv::Mat root(res, CV_16U, cv::Scalar::all(USHRT_MAX));
	cv::Mat visited(res, CV_8U, cv::Scalar::all(0));

	//init
	std::vector< std::pair<float, cv::Point> > open;
	ushort site_id = 0;
	//for every cell
	for (auto& c : this->cells)
	{
		if (debug)
		{
			if (c.site.x<0 || c.site.x>res.height)
				std::cout << "! Warning: c.site.x=" << c.site.x << std::endl;
				

			if (c.site.y<0 || c.site.y>res.width)
				std::cout << "! Warning: c.site.y=" << c.site.y << std::endl;
		}
		
		cv::Point pix((int)c.site.x, (int)c.site.y);
		float d = color2dist(img, pix);

		//stores distances of site
		dist.at<float>(pix.x, pix.y) = d;
		//root stores the site ids
		root.at<ushort>(pix.x, pix.y) = site_id++;
		open.push_back(std::make_pair(d, pix));
		c.coverage.clear();
	}
	//build the max heap with the distance (intensity)
	std::make_heap(open.begin(), open.end(), compareCell);

	//propagate
	while (open.empty() == false)
	{
		//The element with the highest value is moved to (end-1)
		std::pop_heap(open.begin(), open.end(), compareCell);
		// max site (the site that has the highest intensity)
		auto cell = open.back();
		//position of cell
		auto& cpos = cell.second;
		open.pop_back();

		//check if the distance from this cell is already updated
		if (cell.first > dist.at<float>(cpos.x, cpos.y)) continue;
		if (visited.at<uchar>(cpos.x, cpos.y) > 0) continue; //visited
		visited.at<uchar>(cpos.x, cpos.y) = 1;
		// cv::Mat dst;
		// cv::Point2f pt(cpos.x, cpos.y);
		
		// cv::imshow("dst", dst);

		//check the neighbors
		for (int dx =-1; dx <= 1; dx++) //x is row
		{
			//cpos is the site
			
			int x = cpos.x + dx;
			double xf = cpos.x + (double)dx/2;
			if (x < 0 || x >= res.height) continue;
			for (int dy = -1; dy <= 1; dy++) //y is column
			{
				if (dx == 0 && dy == 0) continue; //itself...

				int y = cpos.y + dy;
				double yf = cpos.y + (double)dy/2;
				if (y < 0 || y >= res.width) continue;
				//neighbor(x,y)
                cv::Point pt_tmp(x, y);
				// cv::Point2f tmp (xf,yf);
				// current point's (cpos) intensity + neighbor's intensity
				// minimum-energy configuration
				//we want to minimize the current point's (cpos) intensity + neighbor's intensity (refer to the paper sec 2.1)
				float newd = dist.at<float>(cpos.x, cpos.y) + color2dist(img, pt_tmp);
				
				// float newd = dist.at<float>(cpos.x, cpos.y) + color2dist(resizedImg, pt_tmp);
				//distance of neighbor (** initial distance of neighbor = FLT_MAX)
				float oldd = dist.at<float>(x, y);
				

				if (newd < oldd)
				{
					//set newd (the minimum distance so far) to the distance of neighbor (x,y)
					dist.at<float>(x, y)=newd;
					//set the site id of cpos to its neighbor
					root.at<ushort>(x, y) = root.at<ushort>(cpos.x, cpos.y);
					open.push_back(std::make_pair(newd, cv::Point(x, y)));
					std::push_heap(open.begin(), open.end(), compareCell);
				}
			}//end for dy
		}//end for dx
	}//end while

	//collect cells
	//put the points to the corresponding vor reigions
	for (int x = 0; x < res.height; x++)
	{
		for (int y = 0; y < res.width; y++)
		{
			//site id
			ushort rootid = root.at<ushort>(x, y);
			//points that have the same rootid belong to the same cell
			this->cells[rootid].coverage.push_back(cv::Point(x,y));
		}//end y
	}//end x

	//remove empty cells...
	int cvt_size = this->cells.size();
	for (int i = 0; i < cvt_size; i++)
	{
		if (this->cells[i].coverage.empty())
		{
			this->cells[i] = this->cells.back();
			this->cells.pop_back();
			i--;
			cvt_size--;
		}
	}//end for i

	if (debug)
	{
		//this shows the progress...
		double min;
		double max;
		cv::minMaxIdx(dist, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(dist, adjMap, 255 / max);
		//cv::applyColorMap(adjMap, adjMap, cv::COLORMAP_JET);

		for (auto& c : this->cells)
		{
			cv::circle(adjMap, cv::Point( c.site.y, c.site.x ), 2, CV_RGB(0, 0, 255), -1);
		}

		cv::imshow("CVT", adjMap);
		cv::waitKey(5);
	}

}



void CVT::compute_weighted_cvt(cv::Mat & img, std::vector<cv::Point2d> & sites)
{
	//inint
	int site_size = sites.size();
	this->cells.resize(site_size);
	for (int i = 0; i < site_size; i++)
	{
		this->cells[i].site = sites[i];
	}

	float max_dist_moved = FLT_MAX;

	int iteration = 0;
	do
	{
		vor_sub(img); //compute voronoi
		//maximum distance moved by all sites
		max_dist_moved = move_sites(img);

		if (debug) std::cout << "[" << iteration << "] max dist moved = " << max_dist_moved << std::endl;
		iteration++;
	} while (max_dist_moved>max_site_displacement && iteration < this->iteration_limit);
	
	//if (debug) cv::waitKey();
}
