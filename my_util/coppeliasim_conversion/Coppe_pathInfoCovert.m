function [pathData, pathLen] = Coppe_pathInfoCovert(pathInfo)
%Coppe_pathInfoCovert: convert pathinfo to mat data
N = size(pathInfo{1},2);

pathLen = cell2mat(pathInfo{2}); 

pathData = Coppe_pointsSqueConvert(pathInfo{1}, 7);

% pathData = zeros(7, N);
% for i=1:N
%     for j=1:7
%         pathData(j, i) = pathInfo{1}{i}{j};
%     end
% end