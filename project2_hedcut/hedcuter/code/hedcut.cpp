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



bool Hedcut::build(cv::Mat & input_image, int n)
{
	cv::Mat grayscale;
	cv::cvtColor(input_image, grayscale, cv::COLOR_BGR2GRAY);

	//sample n points
	std::vector<cv::Point2d> pts;
	// sample_initial_points(grayscale, n, pts);
	// HammersleySequence(grayscale, n, pts,0);
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
// void Hedcut:: Hammersley(cv::Mat & img, int n, std::vector<cv::Point2d> & pts){
// 	float p, u, v;
//   	int k, kk, pos;
// 	for (k=0, pos=0 ; k<n ; k++) {
// 	u = 0;
// 	for (p=0.5, kk=k ; kk; p*=0.5, kk>>=1)
// 		if(kk&1)
// 			u += p;
// 	v = (k + 0.5) / n; 
// 	// result[pos++] = u; 
// 	// result[pos++] = v;
// 	}
// }
void Hedcut:: HammersleySequence (cv::Mat & img, int n, std::vector<cv::Point2d> & pts,size_t truncateBits){
	// figure out how many bits we are working in.
	
	cv::RNG rng_gaussian(time(NULL));
    size_t value = 1;
    size_t numBits = 0;
	float x = 0.0f;
	float y = 0.0f;
	cv::Mat visited(img.size(), CV_8U, cv::Scalar::all(0)); //all unvisited
	
    while (value < n)
    {
        value *= 2;
        ++numBits;
    }
 
    // calculate the sample points
	size_t i=0;
    while(i<n){
        // x axis
        // samples[i][0] = 0.0f;
		x = 0.0f;
        {
            size_t m = i >> truncateBits;
            float base = 1.0f / 2.0f;
            while (m)
            {
                if (m & 1){
					// samples[i][0] += base;
					x += base;
				}	
                m /= 2;
                base /= 2.0f;
            }
        }
 
        // y axis
        // samples[i][1] = 0.0f;
		y = 0.0f;
        {
            size_t m = i >> truncateBits;
            size_t mask = size_t(1) << (numBits - 1 - truncateBits);
 
            float base = 1.0f / 2.0f;
            while (mask)
            {
                if (m & mask){
					// samples[i][1] += base;
					y+=base;
				}
                mask /= 2;
                base /= 2.0f;
            }
        }
		size_t r = size_t(x * float(img.size().height));
        size_t c = size_t(y * float(img.size().height));
		std::cout << "c.site.x=" << r << std::endl;
		std::cout << "c.site.y=" << c << std::endl;
		//decide to keep basic on a probability (black has higher probability)
		float value = img.at<uchar>(r, c)*1.0/255; //black:0, white:1
		float gr = fabs(rng_gaussian.gaussian(0.8));
		//rejection sampling
		
		// if ( value < gr) //keep
		// {
			std::cout << "visited=" << visited.at<uchar>(r, c) << std::endl;
			std::cout << "i=" << i << std::endl;
			i++;
			pts.push_back(cv::Point(r, c));
			visited.at<uchar>(r,c)=1;
		// }
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

void Hedcut::edge_init_points(cv::Mat & img, int n, std::vector<cv::Point2d> & pts){
	int count = 0;
	cv::RNG rng_uniform(time(NULL));
	cv::RNG rng_gaussian(time(NULL));
	cv::Mat visited(img.size(), CV_8U, cv::Scalar::all(0)); //all unvisited
	cv::Mat edge, draw;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Canny(img, edge, 50, 150, 3);
	edge.convertTo(draw, CV_8U);
	findContours( edge, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	// imshow("image", draw);
	std::cout<<"size: "<<contours.size()<<std::endl;
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
		disk.center.x = cell.site.y; //x = col
		disk.center.y = cell.site.x; //y = row
		// apply avg r,g,b 
		disk.color = cv::Scalar(r,g,b);
		// disk.color = cv::Scalar::all(0); //black
		disk.radius = 1;

		//remember
		this->disks.push_back(disk);

	}//end for cell

	//done
}
