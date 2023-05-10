function T = Coppe_getMatrix2SE3(getMatrix)
%Coppe_getMatrix2SE3: transform the 12 floats(read as cell array) got from sim.getObjectMatrix
%to SE3
%   Detailed explanation goes here
T = zeros(4,4);
convMatrix = cellfun(@double, getMatrix);

T(1:3, :) = reshape(convMatrix, 3, 4);
T(4,:) = [0,0,0,1];
end

