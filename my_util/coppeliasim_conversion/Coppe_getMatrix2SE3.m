function T = Coppe_getMatrix2SE3(getMatrix)
%Coppe_getMatrix2SE3: transform the 12 floats(read as cell array) got from sim.getObjectMatrix
%to SE3
%   Detailed explanation goes here
T = zeros(4,4);
convMatrix = cell2mat(cellfun(@double, getMatrix));

% for i=1:length(getMatrix) %convert to double
%     convMatrix(i) = double(getMatrix{i});
% end

for i=1:3
    T(i,:) = convMatrix((i-1)*4+1 : (i-1)*4+4);
end
T(4,:) = [0,0,0,1];
end

