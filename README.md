# Smoothing and Mapping using Factor Graph Optimization using GTSAM

This project is an implementation of smoothing and mapping using GTSAM on two datasets:
- 2D Intel dataset
- 3D Parking garage dataset

The data is represented as a factor graph in a `.g2o` file

Broadly two techniques are used for Smoothing:
- Batch optimization (optimize over the whole factor graph at once)
- Incremental optimization (optimize after each edge of the factor graph)

## Usage Instructions

Follow the instructions in the file: https://github.com/nalinbendapudi/Factor-Graph-SLAM-using-GTSAM/blob/master/README%20-%20NalinBendapudi%20Code.txt

## Results

![](https://github.com/nalinbendapudi/Factor-Graph-SLAM-using-GTSAM/blob/master/Results/2D%20Batch%20optimazation.jpg)

Batch Optimization in 2D dataset

![](https://github.com/nalinbendapudi/Factor-Graph-SLAM-using-GTSAM/blob/master/Results/2D%20Incremental%20optimazation.jpg)

Incremental Optimization in 2D dataset

![](https://github.com/nalinbendapudi/Factor-Graph-SLAM-using-GTSAM/blob/master/Results/3D%20Batch%20optimazation%20-%20config1.jpg)

Batch Optimization in 3D dataset

![](https://github.com/nalinbendapudi/Factor-Graph-SLAM-using-GTSAM/blob/master/Results/3D%20Incremental%20optimazation%20-%20config1.jpg)

Incremental Optimization in 3D dataset

