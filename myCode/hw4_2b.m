clc
close all

%% Read from 3D g2o file
filename_3D = '..\data\parking-garage.g2o';
[vertexMatrix, edgeMatrix] = hw4_2a(filename_3D);

%% Initialize NonlinearFactorGraph object (to store edges)
import gtsam.*
graph = NonlinearFactorGraph;

%% Initialize Values object (to store initial estimates of vertices)
initialEstimate = Values;

%% Add prior - Assuming node 0 to be at (0,0,0)
priorMean = Pose3(Rot3(eye(3)),Point3(0,0,0));
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);
graph.add(PriorFactorPose3(0, priorMean, priorNoise));

%% Add all edges to the graph
for i = 1:size(edgeMatrix,1)
    row_i = edgeMatrix(i,:);
    edgeNoise = noiseModel.Gaussian.Covariance(info2Cov(row_i(10:30)));
    t = Point3(row_i(3),row_i(4),row_i(5));
    R = Rot3(quat2rotm([row_i(9),row_i(6),row_i(7),row_i(8)]));
    graph.add(BetweenFactorPose3(row_i(1), row_i(2), Pose3(R,t), edgeNoise));
end

%% Add all vertices to the initialEstimate
for i = 1:size(vertexMatrix,1)
    row_i = vertexMatrix(i,:);
    t = Point3(row_i(2),row_i(3),row_i(4));
    R = Rot3(quat2rotm([row_i(8),row_i(5),row_i(6),row_i(7)]));
    initialEstimate.insert(row_i(1), Pose3(R,t)); 
end

%% Optimize using Gauss-Newton
optimizer = GaussNewtonOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result_poses = utilities.extractPose3(result);

%% Plot the trajectories
figure();
plot3(vertexMatrix(:,2),vertexMatrix(:,3),vertexMatrix(:,4),'b');
hold on
plot3(result_poses(:,10),result_poses(:,11),result_poses(:,12),'r');
legend('initial estimate','optimized trajectory', 'Location','Best')
title('Batch Optimization - 3D Parking Garage dataset')
