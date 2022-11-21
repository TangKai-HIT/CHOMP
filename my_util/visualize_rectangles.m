function visualize_rectangles(ax, rectangles, color)
%compatible with cost map tool box
hold on;
for i =1:length(rectangles)
    width = rectangles(i).high(1) - rectangles(i).low(1);
    height = rectangles(i).high(2) - rectangles(i).low(2);
    rectangle(ax, Position=[rectangles(i).low, width, height], EdgeColor=color, LineStyle='--');
end