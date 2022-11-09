function points = Coppe_pointsSqueConvert(pointsSquence, NumElement)
%Coppe_pointsSqueConvert: convert points squence stored in cell to mat data
N = size(pointsSquence,2);

points = zeros(NumElement, N);
for i=1:N
    for j=1:NumElement
        points(j, i) = pointsSquence{i}{j};
    end
end