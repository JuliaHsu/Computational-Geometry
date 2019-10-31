#include "hedcut.h"
#include "simple_svg_1.0.0.hpp"

using namespace std;

//This variable is defined in wcvt.h
inline string getImageName(const string & img_name)
{
	//
	string output;
	int dot_pos = img_name.rfind(".");
	int slash_pos = img_name.rfind("/");
	if (slash_pos == string::npos)
		output = img_name.substr(0, dot_pos);
	else
		output = img_name.substr(slash_pos + 1, dot_pos - slash_pos - 1);
	//

	return output;
}

int main(int argc, char ** argv)
{

	//get imput image
	if (argc < 2)
	{
		cout << " Usage: " << argv[0] << " [-n #_of_disks -radius disk_radius -iteration #_of_CVT_iterations -maxD max_CVF_site_displacement] image_file_name" << endl;
		return -1;
	}

	Hedcut hedcut;
	Hedcut hedcut_edge;
	bool debug = false;                //output debugging information
	int sample_size = 1000;

	string img_filename;
	for (int i = 1; i < argc; i++)
	{
		if (argv[i][0] == '-')
		{

			if (string(argv[i]) == "-debug") hedcut.debug=debug = true;
			else if (string(argv[i]) == "-n" && i + 1 < argc) sample_size = atoi(argv[++i]);
			else if (string(argv[i]) == "-iteration" && i + 1 < argc) hedcut.cvt_iteration_limit = atoi(argv[++i]);
			else if (string(argv[i]) == "-maxD" && i + 1 < argc) hedcut.max_site_displacement = atof(argv[++i]);
			else if (string(argv[i]) == "-avg" && i + 1 < argc) hedcut.average_termination = true;
			else
				cerr << "! Error: Unknown flag " << argv[i] << ".  Ignored." << endl;
		}
		else img_filename = argv[i];
	}

	cv::Mat image = cv::imread(img_filename.c_str(), cv::IMREAD_COLOR);   // Read the file

	if (!image.data)                              // Check for invalid input
	{
		cout << "! Error: Could not open or find the image" << std::endl;
		return -1;
	}

	if (debug)
	{
		cv::namedWindow("Input image", cv::WINDOW_AUTOSIZE);// Create a window for display.
		imshow("Input image", image);                       // Show our image inside it.
	}

	//
	//compute hedcut
	//
	int w = image.size().width *2;
	int h = image.size().height*2;
	cv::Size res(w, h);
	cv::Mat resizedImg(res.width, res.height, cv::IMREAD_GRAYSCALE);
	cv::resize(image, resizedImg, res, 0, 0, cv::INTER_LINEAR);
	if (hedcut.build(resizedImg, sample_size) == false)
		cerr << "! Error: Failed to build hedcut. Sorry." << endl;

	if (debug)
	{
		cout << "- Generated " << hedcut.getDisks().size() << " disks" << endl;
	}

	// if (hedcut_edge.build_edge(resizedImg, sample_size) == false)
	// 	cerr << "! Error: Failed to build hedcut. Sorry." << endl;

	// if (debug)
	// {
	// 	cout << "- Generated " << hedcut_edge.getDisks().size() << " disks" << endl;
	// }

	//
	//save output to svg
	//
	stringstream ss;
	string img_name = getImageName(img_filename);
	ss << img_name << "-" << sample_size << ".svg";
	svg::Dimensions dimensions(image.size().width,image.size().height);
	svg::Document doc(ss.str(), svg::Layout(dimensions, svg::Layout::TopLeft));
	for (auto& disk : hedcut.getDisks())
	{
		uchar r = disk.color.val[0];
		uchar g = disk.color.val[1];
		uchar b = disk.color.val[2];
		svg::Color color(r, g, b);
		svg::Circle circle(svg::Point(disk.center.x, disk.center.y), disk.radius * 2, svg::Fill(color));
		doc << circle;
	}//end for i
	// for(auto & disk: hedcut_edge.getDisks()){
	// 	uchar r = disk.color.val[0];
	// 	uchar g = disk.color.val[1];
	// 	uchar b = disk.color.val[2];
	// 	svg::Color color(r, g, b);
	// 	svg::Circle circle(svg::Point(disk.center.x, disk.center.y), disk.radius * 2, svg::Fill(color));
	// 	doc<<circle;
	// }

	doc.save();

	if (debug)
	{
		cout << "- Saved " << ss.str() << endl;
	}
	return 0;
}