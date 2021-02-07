clc
close all

%% Read from 2D g2o file
filename_2D = '..\data\input_INTEL_g2o.g2o';
[vertexMatrix, edgeMatrix] = hw4_1a(filename_2D);

%% Initialize NonlinearFactorGraph object (to store edges)
import gtsam.*
graph = NonlinearFactorGraph;

%% Initialize Values object (to store initial estimates of vertices)
initialEstimate = Values;

%% Add prior - Assuming node 0 to be at (0,0,0)
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(0, Pose2(0, 0, 0), priorNoise));

%% Add all edges to the graph
for i = 1:size(edgeMatrix,1)
    row_i = edgeMatrix(i,:);
    edgeNoise = noiseModel.Gaussian.Covariance(info2Cov(row_i(6:11)));
    graph.add(BetweenFactorPose2(row_i(1), row_i(2), Pose2(row_i(3),row_i(4),row_i(5)), edgeNoise));
end

%% Add all vertices to the initialEstimate
for i = 1:size(vertexMatrix,1)
   row_i = vertexMatrix(i,:);
   initialEstimate.insert(row_i(1), Pose2(row_i(2),row_i(3),row_i(4))); 
end

%% Optimize using Gauss-Newton
optimizer = GaussNewtonOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result_poses = utilities.extractPose2(result);

%% Plot the trajectories
figure();
plot(vertexMatrix(:,2),vertexMatrix(:,3),'b');
hold on
plot(result_poses(:,1),result_poses(:,2),'r');
legend('initial estimate','optimized trajectory', 'Location','Best')
title('Batch Optimization - 2D INTEL dataset')
