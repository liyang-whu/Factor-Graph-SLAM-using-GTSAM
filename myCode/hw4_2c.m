clc
close all

%% Read from 2D g2o file
filename3D = '..\data\parking-garage.g2o';
[vertexMatrix, edgeMatrix] = hw4_2a(filename3D);

%% Import GTSAM library
import gtsam.*

%% Initialize ISAM2 object
isam = ISAM2;

%% Sort Edges so that we can add them incrementally
sortedEdgeMatrix = sortrows(edgeMatrix,2);

%% Seperate the odometry edges
odometryEdgeMatrix = edgeMatrix( edgeMatrix(:,1)+1 == edgeMatrix(:,2), : );
odometryEdgeMatrix = sortrows(odometryEdgeMatrix,2);

%% First iteration - Node 0 and prior edge

% Initialize objects from graph and initialEstimate
graph = NonlinearFactorGraph;
initialEstimate = Values;

% Add prior - Assuming node 0 to be at (0,0,0)
priorMean = Pose3(Rot3(eye(3)),Point3(0,0,0));
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);
graph.add(PriorFactorPose3(0, priorMean, priorNoise));

% Add Node 0
t = Point3(vertexMatrix(1,2),vertexMatrix(1,3),vertexMatrix(1,4));
R = Rot3(quat2rotm([vertexMatrix(1,8),vertexMatrix(1,5),vertexMatrix(1,6),vertexMatrix(1,7)]));
initialEstimate.insert(vertexMatrix(1,1), Pose3(R,t));
    
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
    newNode_x = result.at(vertexIndex-1).x + odometryEdgeMatrix(vertexIndex,3);
    newNode_y = result.at(vertexIndex-1).y + odometryEdgeMatrix(vertexIndex,4);
    newNode_z = result.at(vertexIndex-1).z + odometryEdgeMatrix(vertexIndex,5);  
    newNode_t = Point3(newNode_x,newNode_y,newNode_z);
    edge_R    = Rot3(quat2rotm([odometryEdgeMatrix(vertexIndex,9),odometryEdgeMatrix(vertexIndex,6),odometryEdgeMatrix(vertexIndex,7),odometryEdgeMatrix(vertexIndex,8)]));
    newNode_R = result.at(vertexIndex-1).rotation.compose(edge_R);
    initialEstimate.insert(vertexIndex, Pose3(newNode_R,newNode_t));

    % Add edges to the graph
    while sortedEdgeMatrix(edgeIndex,2) == vertexIndex
        row_i = sortedEdgeMatrix(edgeIndex,:);
        edgeNoise = noiseModel.Gaussian.Covariance(info2Cov(row_i(10:30)));
        t = Point3(row_i(3),row_i(4),row_i(5));
        R = Rot3(quat2rotm([row_i(9),row_i(6),row_i(7),row_i(8)]));
        graph.add(BetweenFactorPose3(row_i(1), row_i(2), Pose3(R,t), edgeNoise));
        edgeIndex = edgeIndex+1;
        if edgeIndex > size(sortedEdgeMatrix,1)
            break
        end
    end
    
    % Update isam and calculate result
    isam.update(graph,initialEstimate); 
    result = isam.calculateEstimate;
end

result_poses = utilities.extractPose3(result);

%% Plot the trajectories
figure();
plot3(vertexMatrix(:,2),vertexMatrix(:,3),vertexMatrix(:,4),'b');
hold on
plot3(result_poses(:,10),result_poses(:,11),result_poses(:,12),'r');
legend('initial estimate','optimized trajectory', 'Location','Best')
title('Incremental Optimization - 3D Parking Garage dataset')
