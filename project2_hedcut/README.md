# Project 2: hedcut generator

## The first goal of this project is to understand and summary the implementation of creating stipple drawing by Centroidal Voronoi Tessellation from the paper by Secord, Adrian. “Weighted voronoi stip- pling.” <br/>
The second of this project is improve the method via multiple ways.

The report of this project is enclosed in the report folder, which contains the summary of the implementation and brief desciprtions of my improvents.

## The improvements I made:

1.Improve the distribution of the disks with subpixel
![image](picture or gif url)

2. Extract features of input image

3. Generate colorful disks

4. Adjust the radi of the disks by considering the contrast of the image


  
To run the code, try this in hedcuter folder:

> ./code/build/hedcuter  -n 10000 -iteration 10 images/einstein-medium.jpg

This should create a file called ``einstein-medium-10000.svg".
