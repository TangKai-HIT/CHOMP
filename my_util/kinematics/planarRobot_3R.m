classdef planarRobot_3R<handle
    %PLANARROBOT_3R simple 3R robot struct for motion planning
    %   
    
    properties
        linkLength;
        homeConfig = [0,0,0];
        sampledPoints;
        endPointPosition;
        jointsPosition = cell(1,3);
        base = [0,0];

        config; %N x  dim
%         endPointTraj;
    end
    
    properties(SetAccess=private, GetAccess=private)
%         sampledPointStruct = struct("linkId",[], "localPosition",[], "globalPosition",[]);
        Rot2D = @(t) [cos(t), -sin(t); sin(t), cos(t)];
    end

    methods
        function obj = planarRobot_3R(linkLength, base)
            %PLANARROBOT_3R 构造此类的实例
            %   此处显示详细说明
            obj.linkLength = linkLength;
            obj.base = base;
        end
        
        function addSamplePtOnLink(obj, sampleLinkId, numSample)
            %ADDSAMPLEPTONLINK 此处显示有关此方法的摘要
            %   此处显示详细说明
            Id=1;
            for i=1:length(sampleLinkId)
                interval = obj.linkLength(i)/numSample(i);
                for j=1:numSample(i)          
                    obj.sampledPoints(Id).linkId = sampleLinkId(i);
                    obj.sampledPoints(Id).localPosition = [j*interval, 0];

                    Id = Id+1;
                end
            end
        end

        function updateAllPtPosition(obj, config)
            %UPDATEALLPTPOSITION
            if exist("config", "var")
                obj.config = config;
            end
            N = size(obj.config, 1);
            obj.endPointPosition = zeros(N,2);

            obj.updateSamplePtPosition();
            for i=1:N
                obj.endPointPosition(i,:) = obj.jointsPosition{3}(i, :) ...
                                                + (obj.Rot2D(sum(obj.config(i, :))) * [obj.linkLength(3);0])';
            end
        end
       
        function bodyPts = updateSamplePtPosition(obj, config)
            %GETSAMPLEPTPOSITION 此处显示有关此方法的摘要
            %   此处显示详细说明  
            if exist("config", "var")
                obj.config = config;
            end
            N = size(obj.config, 1);
            obj.updateJointPosition();
            numSamplePts = length(obj.sampledPoints);
            bodyPts = cell(1, numSamplePts);
            
            for i=1:numSamplePts
                obj.sampledPoints(i).globalPosition = zeros(N, 2);
                for j=1:N
                    switch obj.sampledPoints(i).linkId
                        case 1
                            obj.sampledPoints(i).globalPosition(j, :) = obj.jointsPosition{1}(j, :) ...
                                                + (obj.Rot2D(obj.config(j, 1)) * obj.sampledPoints(i).localPosition')';
                        case 2
                            obj.sampledPoints(i).globalPosition(j, :) = obj.jointsPosition{2}(j, :) ...
                                                + (obj.Rot2D(obj.config(j, 1)+obj.config(j, 2)) * obj.sampledPoints(i).localPosition')';
                        case 3
                            obj.sampledPoints(i).globalPosition(j, :) = obj.jointsPosition{3}(j, :) ...
                                                + (obj.Rot2D(sum(obj.config(j, :))) * obj.sampledPoints(i).localPosition')';
                    end
                end
                bodyPts{i} = obj.sampledPoints(i).globalPosition;
            end
        end

        function jointsPosition = updateJointPosition(obj)
            %GETJOINTPOSITION 此处显示有关此方法的摘要
            %   此处显示详细说明
            N = size(obj.config, 1);           
            for i =1:3
                obj.jointsPosition{i} = zeros(N, 2);
                for j=1:N
                    switch i
                        case 1
                            obj.jointsPosition{i}(j, :) = obj.base;
                        case 2
                            obj.jointsPosition{i}(j, :) = obj.jointsPosition{i-1}(j, :) + (obj.Rot2D(obj.config(j, 1)) * [obj.linkLength(i); 0])';
                        case 3
                            obj.jointsPosition{i}(j, :) =obj.jointsPosition{i-1}(j, :) + (obj.Rot2D(obj.config(j, 1)+obj.config(j, 2)) * [obj.linkLength(i); 0])';
                    end
                end
            end
            jointsPosition = obj.jointsPosition{i};
        end

        function bodyJacobian = getSamplePtJacobian(obj)
            %GETSAMPLEPTJACOBIAN
            N = size(obj.config, 1); 
            numSamplePts = length(obj.sampledPoints);
            bodyJacobian = cell(1, numSamplePts);
            I_tilt = [0 -1; 1 0]; 
            O_21 = zeros(2,1);

            for i=1:numSamplePts
                jacobian = zeros(2,3,N);
                for j=1:N
                    vec_r1 = obj.sampledPoints(i).globalPosition(j, :) - obj.jointsPosition{1}(j, :);
                    colunm1 = I_tilt * vec_r1';
                    switch obj.sampledPoints(i).linkId
                        case 1
                            jacobian(:, :, j) = [colunm1  , O_21, O_21];
                        case 2
                            vec_r2 = obj.sampledPoints(i).globalPosition(j, :) - obj.jointsPosition{2}(j, :);
                            colunm2 = I_tilt * vec_r2';
                             jacobian(:, :, j) = [colunm1, colunm2, O_21];
                        case 3
                            vec_r2 = obj.sampledPoints(i).globalPosition(j, :) - obj.jointsPosition{2}(j, :);
                            colunm2 = I_tilt * vec_r2';
                            vec_r3 = obj.sampledPoints(i).globalPosition(j, :) - obj.jointsPosition{3}(j, :);
                            colunm3 = I_tilt * vec_r3';
                            jacobian(:, :, j) = [colunm1, colunm2, colunm3];
                    end
                end
                bodyJacobian{i} = jacobian;
            end
        end
        
        function plotRobot2D(obj, ax, color, slice)
            %PLOTROBOT2D 
            X = zeros(1,3);
            Y = zeros(1, 3);
            
            if ~exist("color", "var")
                color = 'black';
            end
            
            hold on;

             if exist("slice","var")
                Index = slice;
                if length(slice)>1
                    step = 1:(slice(end)/(slice(2)-slice(1)));
                else
                    step = slice;
                end
            else
                Index = 1:size(obj.config, 1);
                step=index;
            end

            for n = step
                if ischar(color) || isstring(color)
                    plotColor = color;
                else
                    plotColor = color(n, :);
                end

                for i=1:3
                    X(i) = obj.jointsPosition{i}(Index(n) ,1);
                    Y(i) = obj.jointsPosition{i}(Index(n) ,2);

                    plot(ax, X(i), Y(i), 'o', 'MarkerSize',10, 'Color', plotColor, 'MarkerFaceColor', plotColor);
                    if i>1
                        plot(ax, X(i-1:i), Y(i-1:i), "LineStyle",'-',"Color",plotColor, 'MarkerSize',6);
                    end
                end
                
                 plot(ax, obj.endPointPosition(Index(n),1), obj.endPointPosition(Index(n),2), 'o', 'MarkerSize',10, 'Color', plotColor, 'MarkerFaceColor', plotColor);
                 plot(ax, [obj.endPointPosition(Index(n),1), obj.jointsPosition{3}(Index(n) ,1)], [obj.endPointPosition(Index(n),2), obj.jointsPosition{3}(Index(n) ,2)],...
                        "LineStyle",'-',"Color",plotColor, 'MarkerSize',6);

            end
        end

        function plotSampledPt(obj, ax, color, slice)
            %PLOTSAMPLEDPT
            hold on;

            if exist("slice","var")
                Index = slice;
                if length(slice)>1
                    step = 1:(slice(end)/(slice(2)-slice(1)));
                else
                    step = slice;
                end
            else
                Index = 1:size(obj.config, 1);
                step=index;
            end

            for n = step
                if ischar(color) || isstring(color)
                    plotColor = color;
                else
                    plotColor = color(n, :);
                end
                for i=1:length(obj.sampledPoints)
                    x =obj.sampledPoints(i).globalPosition(Index(n), 1);
                    y =obj.sampledPoints(i).globalPosition(Index(n), 2);
                    plot(ax, x, y, 'o', 'MarkerSize',6, 'Color', plotColor, 'MarkerFaceColor',plotColor); 
                end
            end
        end

    end

end

