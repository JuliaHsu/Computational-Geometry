#include "hedcut.h"
#include <time.h>


Hedcut::Hedcut()
{
	//cvt control flags
	cvt_iteration_limit = 100; //max number of iterations when building cvf
	max_site_displacement = 1.01f; //max tolerable site displacement in each iteration.
	average_termination = false;

	debug = false;
}

// bool Hedcut::build_edge(cv::Mat & input_image, int n){
// 	cv::Mat grayscale;
// 	cv::cvtColor(input_image, grayscale, cv::COLOR_BGR2GRAY);
// 	std::vector<cv::Point2d> pts;
// 	edge_init_points(grayscale, n, pts);
// 	CVT cvt;
// 	cvt.iteration_limit = this->cvt_iteration_limit;
// 	cvt.max_site_displacement = this->max_site_displacement;
// 	cvt.average_termination = this->average_termination;
// 	cvt.debug = this->debug;

// 	clock_t startTime, endTime;
// 	startTime = clock();

// 	//compute weighted centroidal voronoi tessellation
// 	cvt.compute_weighted_cvt(grayscale, pts);	//*****

// 	endTime = clock();
// 	std::cout << "Total time: "<< ((double)(endTime - startTime)) / CLOCKS_PER_SEC << std::endl;

// 	if (debug) cv::waitKey();

// 	//create disks
// 	create_disks(input_image, cvt);

// 	return true;
// }

bool Hedcut::build(cv::Mat & input_image, int n)
{
	cv::Mat grayscale;
	cv::cvtColor(input_image, grayscale, cv::COLOR_BGR2GRAY);

	//sample n points
	std::vector<cv::Point2d> pts;
	// sample_initial_points(grayscale, n, pts);
	edge_init_points(grayscale, n, pts);

	//initialize cvt
	CVT cvt;

	cvt.iteration_limit = this->cvt_iteration_limit;
	cvt.max_site_displacement = this->max_site_displacement;
	cvt.average_termination = this->average_termination;
	cvt.debug = this->debug;

	clock_t startTime, endTime;
	startTime = clock();

	//compute weighted centroidal voronoi tessellation
	cvt.compute_weighted_cvt(grayscale, pts);	//*****

	endTime = clock();
	std::cout << "Total time: "<< ((double)(endTime - startTime)) / CLOCKS_PER_SEC << std::endl;

	if (debug) cv::waitKey();

	//create disks
	create_disks(input_image, cvt);

	return true;
}



void Hedcut::edge_init_points(cv::Mat & img, int n, std::vector<cv::Point2d> & pts){
	int count = 0;
	int countour_size = 0;

	cv::RNG rng_uniform(time(NULL));
	cv::RNG rng_gaussian(time(NULL));
	cv::Size res(img.size().width, img.size().height);
	cv::Mat visited(res, CV_8U, cv::Scalar::all(0)); //all unvisited
	cv::Mat edge, draw;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	// cv::Canny(img, edge, 50, 150, 3);
	cv::Canny(img, edge, 50, 150, 3);
	
	findContours( edge, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	// imshow("image", draw);
	
	for(int i = 0; i < contours.size() && count<n; i++){
		for(int j = 0; j < contours[i].size() && count<n;j+=10){
			countour_size++;
		}
	}
	if(n>countour_size){
		for(int i = 0; i < contours.size() && count<n; i++){
			for(int j = 0; j < contours[i].size() && count<n;j+=10){
				int c = contours[i][j].x;					
				int r = contours[i][j].y;
				if (visited.at<uchar>(r, c) ==0){
					count++;
					pts.push_back(cv::Point(r, c));
					visited.at<uchar>(r,c)=1;
				}
			}
  		}
	}
	
	std::cout<<"count: "<<count<< std::endl;
	while (count < n)
	{
		//generate a random point, uniform distribution
			int c = (int)floor(img.size().width*rng_uniform.uniform(0.f, 1.f));
			int r = (int)floor(img.size().height*rng_uniform.uniform(0.f, 1.f));
			//decide to keep basic on a probability (black has higher probability)
			float value = img.at<uchar>(r, c)*1.0/255; //black:0, white:1
			float gr = fabs(rng_gaussian.gaussian(0.8));
			//rejection sampling
			
			if ( value < gr && visited.at<uchar>(r, c) ==0) //keep
			{
				count++;
				pts.push_back(cv::Point(r, c));
				visited.at<uchar>(r,c)=1;
			}
		
		
		
	}

	if (debug)
	{
		cv::Mat tmp = img.clone();
		for (auto& c : pts)
		{
			cv::circle(tmp, cv::Point(c.y, c.x), 2, CV_RGB(0, 0, 255), -1);
		}
		cv::imshow("samples", tmp);
		cv::waitKey();
	}
}

void Hedcut::sample_initial_points(cv::Mat & img, int n, std::vector<cv::Point2d> & pts)
{
	//create n points that spread evenly that are in areas of black points...
	int count = 0;

	cv::RNG rng_uniform(time(NULL));
	cv::RNG rng_gaussian(time(NULL));
	cv::Mat visited(img.size(), CV_8U, cv::Scalar::all(0)); //all unvisited
	while (count < n)
	{
		//generate a random point, uniform distribution
		int c = (int)floor(img.size().width*rng_uniform.uniform(0.f, 1.f));
		int r = (int)floor(img.size().height*rng_uniform.uniform(0.f, 1.f));
		//decide to keep basic on a probability (black has higher probability)
		float value = img.at<uchar>(r, c)*1.0/255; //black:0, white:1
		float gr = fabs(rng_gaussian.gaussian(0.8));
		//rejection sampling
		
		if ( value < gr && visited.at<uchar>(r, c) ==0) //keep
		{
			count++;
			pts.push_back(cv::Point(r, c));
			visited.at<uchar>(r,c)=1;
		}
	}

	if (debug)
	{
		cv::Mat tmp = img.clone();
		for (auto& c : pts)
		{
			cv::circle(tmp, cv::Point(c.y, c.x), 2, CV_RGB(0, 0, 255), -1);
		}
		cv::imshow("samples", tmp);
		cv::waitKey();
	}
}

void Hedcut::create_disks(cv::Mat & img, CVT & cvt)
{
	cv::Mat grayscale;
	cv::cvtColor(img, grayscale, cv::COLOR_BGR2GRAY);

	disks.clear();
	int minSize, maxSize,c, totalSize;
	minSize = img.size().width * img.size().height;
	maxSize = 0;
	totalSize =0;
	c =0;
	std::cout<<"min: "<<minSize<<std::endl;
	std::cout<<"max: "<<maxSize<<std::endl;
	for (auto& cell : cvt.getCells()){
		totalSize+= cell.coverage.size();
		c++;
		if(cell.coverage.size()>maxSize){
			maxSize = cell.coverage.size();
		}
		if(cell.coverage.size()<minSize){
			minSize = cell.coverage.size();
		}		
		
	}
	float mean = totalSize /c;
	float stdd = 0.0f;
	for (auto& cell : cvt.getCells()){
		stdd += pow(cell.coverage.size() - mean,2);
	}
	stdd = sqrt(stdd / c);
	
	//create disks from cvt
	for (auto& cell : cvt.getCells())
	{
		//compute avg intensity
		unsigned int total = 0;
		unsigned int r = 0, g = 0, b = 0;
		for (auto & resizedPix : cell.coverage)
		{
			cv::Point pix(resizedPix.x, resizedPix.y);
			total += grayscale.at<uchar>(pix.x, pix.y);
			r += img.at<cv::Vec3b>(pix.x, pix.y)[2];
			g += img.at<cv::Vec3b>(pix.x, pix.y)[1];
			b += img.at<cv::Vec3b>(pix.x, pix.y)[0];
		}
		float avg_v = floor(total * 1.0f/ cell.coverage.size());
		// get the average r, g, b of the points that are in the same cell 
		r = floor(r / cell.coverage.size());
		g = floor(g / cell.coverage.size());
		b = floor(b / cell.coverage.size());
	
		//create a disk
		HedcutDisk disk;
		disk.center.x = (double)(cell.site.y)/2; //x = col
		// std::cout<<"center: "<<disk.center.x<<std::endl;
		disk.center.y = (double)(cell.site.x)/2; //y = row
		// apply avg r,g,b 
		disk.color = cv::Scalar(r,g,b);
		// disk.color = cv::Scalar::all(0); //black
		disk.radius = 1;
		// disk.radius =(256.0f - avg_v)/255.0f;
		// disk.radius =1000/abs((cell.coverage.size()-mean))/(stdd);
		// disk.radius = 10000*(cell.coverage.size()-minSize)/(maxSize-minSize);
		// disk.radius = disk.radius/10000;
		// std::cout<<"r = "<<disk.radius<<std::endl;
		// std::cout<<"r = "<<disk.radius<<std::endl;

		//remember
		this->disks.push_back(disk);

	}//end for cell

	//done
}
