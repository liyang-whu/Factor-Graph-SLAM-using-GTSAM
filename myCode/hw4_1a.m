function [vertex_matrix, edge_matrix] = hw4_1a(filename)
fid = fopen(filename);
C = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f %f');
fid = fclose(fid);
numberLines = size(C{1},1);
A = [C{2} C{3} C{4} C{5} C{6} C{7} C{8} C{9} C{10} C{11} C{12}];

vertex_indices = [];
edge_indices = [];

for i = 1:numberLines 
    if strcmp('VERTEX_SE2',C{1}{i})
        vertex_indices = [vertex_indices;i];
    elseif strcmp('EDGE_SE2',C{1}{i})
        edge_indices = [edge_indices;i];
    else
        disp('new type of variable!! Cannot be parsed');
    end
end

vertex_matrix = A(vertex_indices,(1:4));
edge_matrix = A(edge_indices,(1:11));
end
