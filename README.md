# hunky-dory

Change detection for large point clouds.

![Changes](https://upload.wikimedia.org/wikipedia/en/thumb/e/e8/Bowiechanges2.jpg/220px-Bowiechanges2.jpg)

Right now this takes the form of a single executable, written in C++, that brings together [PDAL](http://www.pdal.io/), [entwine](https://entwine.io/#/), [CPD](https://github.com/gadomski/cpd), and [PCL's ICP](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php).
Build two **entwine** indices, then point **hunky-dory** at those indices to perform piecewise (i.e. segmented) change detection.
