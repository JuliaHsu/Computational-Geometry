# Project 2: Hedcut Generator

## The first goal of this project is to understand and summary the implementation of creating stipple drawing by Centroidal Voronoi Tessellation from the paper by Secord, Adrian. “Weighted voronoi stip- pling.” <br/>
The second of this project is improve the method via multiple ways.

The report of this project is enclosed in the report folder, which contains the summary of the implementation and brief desciprtions of my improvents.

## The improvements I made:

1.Improve the distribution of the disks with subpixel

![Original Image](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry3.jpg)
![without using subpixel](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry3-10000_random.png)
![improvement with subpixel](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry3-10000_random_sub.png)


2. Extract features of input image
![Original Image](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry2.jpg)
![without Canny Edge Detector](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry2_random.png)
![with Canny Edge Detector](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry2_edge.png)

3. Generate colorful disks

![Original Image](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry2.jpg)
![Color Disks](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry2_color.png)
![Original Image](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry1.jpg)
![Color Disks](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry1_color.png)

4. Adjust the radi of the disks by considering the contrast of the image
![ Stipples with different size of disks](https://github.com/JuliaHsu/Computational-Geometry/blob/master/project2_hedcut/report/image/harry1_contrast.png)


  
To run the code, try this in hedcuter folder:

> ./code/build/hedcuter  -n 10000 -iteration 10 images/einstein-medium.jpg

This should create a file called ``einstein-medium-10000.svg".
