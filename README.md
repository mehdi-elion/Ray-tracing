# Ray-tracing
A set of C++ codes to render a scene using stochastic ray tracing.

## Overview
This repository contains codes that can be used to :
<ul>
    <li>define a scene with objects having their own optical properties</li>
    <li>and render this scene (as an image) with stochastic ray casting</lI>
</ul>

Each object can be a sphere, a triangular face, or a more complex object defined by a triangular mesh (i.e. an ensemble of triangular faces).

<img src="./Lancer de Rayon/archive/1024px-Ray_trace_diagram.png" width="500" 
     height="auto" class="center">

Ray casting works by tracing a path from an imaginary eye through each pixel in a virtual screen, and calculating the color of the object visible through it. In our case, we use the recursive version of that algorithm where each cast ray generates refracted or reflected rays which will be taken into account to compute tje color of the pixel.<br>


## Features
Here a brief and visual summary of the different features available in this code. <br>

#### Direct and indirect lighting (Monte-Carlo)
Each object can be illuminated in two ways :
<ul>
    <li>through direct lighting, i.e. with light coming from the light source</li>
    <li>through indirect lighting, i.e. with light reflected by intermediary objects</li>
</ul>
The indirect component is computed by Monte-Carlo integration of stochastic rays, i.e. rays cast in random directions. 



Direct lighting only          |  Direct & Indirect lighting
:----------------------------:|:----------------------------:
<img src="./Lancer de Rayon\archive\misc\image_ombre.png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon/archive/misc/image - OK.png" style="width:100%" class="center">


Also note the importance of the number of stochastic rays (N) in the monte-carlo integration and the influence it has on the quality of the rendered image.


4 rays                        |  2000 rays
:----------------------------:|:----------------------------:
<img src="./Lancer de Rayon\archive\misc\BG0 - nbRays=4 nbReb=4 time=0h0m6s.png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon\archive\misc\BG0 - nbRays=2000 nbReb=4 time=1h6m55s.png" style="width:100%" class="center">

<br>



#### Expanded light source
Having a spatially extended light source (instead of a punctual one) causes shadows to be softer. In our case, we modeled that by considering that the light source was a sphere oy radius **R_lum**.  

Expanded source, R_lum=2          |  Expanded light source, R_lum=6
:--------------------------------:|:--------------------------------:
<img src="./Lancer de Rayon\archive\R_lum\R_lum=2 nbRays=1000 nbReb=4 time=0h4m1s.png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon\archive\R_lum\R_lum=6 nbRays=1000 nbReb=4 time=0h3m58s.png" style="width:100%" class="center">

<br>


#### Anti-aliasing
There are several methods to implement anti-aliasing. We opted for one that consists in casting N rays from random positions inside each pixel instead of having all of them cast from pixel centers. 

Without anti-aliasing             |  With anti-aliasing
:--------------------------------:|:--------------------------------:
<img src="./Lancer de Rayon\archive\without_antialiasing.png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon\archive\with_antialiasing.png" style="width:100%" class="center">

<br>



#### Phong's smoothing
Phong's smoothing is a technique that renders smoother meshes. It consists in interpolating, for each pixel, surface normals of triangular elements. 


Without Phong's smoothing         |  With Phong's smoothing
:--------------------------------:|:--------------------------------:
<img src="./Lancer de Rayon/archive/Dossier Rapport\nbRays=1000 nbReb=4 time=0h31m51s (sans Phong).png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon/archive/Dossier Rapport\nbRays=1000 nbReb=4 time=0h33m11s (avec Phong).png" style="width:100%" class="center">
<br>


#### Depth of field
To simulate blur in our images, we chose to model a pinhole camera, which are made of a lens and a shutter.


Default camera                    |  Pinhole camera
:--------------------------------:|:--------------------------------:
<img src="./Lancer de Rayon/archive/Dossier Rapport\nbRays=1000 nbReb=4 time=0h3m9s.png" style="width:100%" class="center">  |  <img src="./Lancer de Rayon/archive/Dossier Rapport\image_recovery.png" style="width:100%" class="center">

<br>


## How to use it
To use this code, first make sure that you have a C++ compiler. Then, we advise you to use Visual Code Studio. You can open the project from Visual Studio Code using the <code>Lancer de Rayon\Lancer de Rayon.sln</code> file.Finally, generate the solution and run the code to render the image that was defined in the <code>Lancer de Rayon\Lancer de Rayon\Source.cpp</code> script. 