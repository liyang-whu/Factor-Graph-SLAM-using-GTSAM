clc
close all

%% Read from 2D g2o file
filename2D = '..\data\input_INTEL_g2o.g2o';
[vertexMatrix, edgeMatrix] = hw4_1a(filename2D);

%% Import GTSAM library
import gtsam.*

%% Initialize ISAM2 object
isam = ISAM2;

%% Sort Edges so that we can add them incrementally
sortedEdgeMatrix = sortrows(edgeMatrix,2);

%% First iteration - Node 0 and prior edge

% Initialize objects from graph and initialEstimate
graph = NonlinearFactorGraph;
initialEstimate = Values;

% Add prior - Assuming node 0 to be at (0,0,0)
priorMean = Pose2(0, 0, 0);
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise));

% Add Node 0
initialEstimate.insert(0, Pose2(vertexMatrix(1,2),vertexMatrix(1,3),vertexMatrix(1,4)));

% Update isam and calculate result
isam.update(graph,initialEstimate);
result = isam.calculateEstimate;

%% Add one vertex and corresponding edges and optimize at each step
edgeIndex = 1;
for vertexIndex = 1:size(vertexMatrix,1)-1
    
    % Re-initialize or clear graph and initialEstimate
    initialEstimate.clear();
    graph = NonlinearFactorGraph;
    
    % Calculate initial pose of new node
    newNode_x = result.at(vertexIndex-1).x + edgeMatrix(vertexIndex,3);
    newNode_y = result.at(vertexIndex-1).y + edgeMatrix(vertexIndex,4);
    newNode_theta = result.at(vertexIndex-1).theta + edgeMatrix(vertexIndex,5);  
    
    initialEstimate.insert(vertexIndex, Pose2(newNode_x,newNode_y,newNode_theta));

    % Add edges to the graph
    while sortedEdgeMatrix(edgeIndex,2) == vertexIndex
        row_i = sortedEdgeMatrix(edgeIndex,:);
        edgeNoise = noiseModel.Gaussian.Covariance(info2Cov(row_i(6:11)));
        graph.add(BetweenFactorPose2(row_i(1), row_i(2), Pose2(row_i(3),row_i(4),row_i(5)), edgeNoise));
        edgeIndex = edgeIndex+1;
        if edgeIndex > size(sortedEdgeMatrix,1)
            break
        end
    end
    
    % Update isam and calculate result
    isam.update(graph,initialEstimate); 
    result = isam.calculateEstimate;
end

result_poses = utilities.extractPose2(result);

%% Plot the trajectories
figure();
plot(vertexMatrix(:,2),vertexMatrix(:,3),'b');
hold on
plot(result_poses(:,1),result_poses(:,2),'r');
legend('initial estimate','optimized trajectory', 'Location','Best')
title('Incremental Optimization - 2D INTEL dataset')
