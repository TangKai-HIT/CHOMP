classdef chomp_wp_planner < handle
    %CHOMP_WP_PLANNER way point planner using chomp 
    %   Detailed explanation goes here

    properties
        options
        constraints

        num_pt % number of way points to be optimized
        dim % dimension of way points
        smooth_order % 1~3
        init_traj

    end

     properties(SetAccess=private)
        optim_traj % way points to be optimized
        boudary_pt_Id = cell(1,2) % index of boundary way points
        cost_func % final cost function for optimization
        grad_func % final gradient function for optimization
        solverLog = struct('history', [], 'iterations', [])% output log
    end

    methods
        function obj = chomp_wp_planner(init_traj, smooth_order, isEndPtFixed)
            %CHOMP_WP_PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            addpath(genpath(pwd));
            obj.options = chomp_optimset_init();
            
            obj.smooth_order = smooth_order;

            obj.init_traj = init_traj;
            obj.optim_traj = init_traj; % n X dim
            
            obj.dim = size(init_traj, 2);
            obj.constraints = chomp_set_constraints(obj.dim);

            if isEndPtFixed % fixed end point?
                obj.num_pt = size(init_traj, 1) - 2;
                obj.boudary_pt_Id{1} = 1;
                obj.boudary_pt_Id{2} =  size(init_traj, 1);
                obj.options.FreeEndPoint = 0;
            else
                obj.num_pt = size(init_traj, 1) - 1;
                obj.boudary_pt_Id{1} = 1;
                obj.options.FreeEndPoint = 1;
            end

        end
        
        function set_cost_grad_fn(obj, costfnHandles, gradfnHandles)
            obj.cost_func = costfnHandles;
            obj.grad_func = gradfnHandles;
        end

        function [optim_traj, cost_best, exitflag, output] = solve(obj)
            %SOLVE iteratively solve the squential quadratic programming problem
            %   Detailed explanation goes here
            if isempty(obj.constraints.C) && isempty(obj.constraints.lb) && isempty(obj.constraints.ub)
                [optim_traj , cost_best, exitflag, output ] = chomp_CovGradDescent( obj.init_traj, obj.boudary_pt_Id, ...
                                                                                                         obj.cost_func, obj.grad_func, obj.constraints, obj.options );
            end
            
            obj.solverLog = output;
            obj.optim_traj = optim_traj;
        end
    
        function deformHistory_plot(obj, ax, cmapHandle)
            %DEFORMHISTORY_PLOT plot history of 2D/3D trajectory deformation by a
            %color map
            numHistory = length(obj.solverLog.history);

            if obj.dim == 2
                hold(ax, "on");
                cc=cmapHandle(numHistory);
                for i = 1:numHistory
                    plot(ax, obj.solverLog.history(i).xi(:,1), obj.solverLog.history(i).xi(:,2),'color',cc(i,:));
                end

            elseif obj.dim == 3
                hold(ax, "on");
                cc=cmapHandle(numHistory);
                for i = 1:numHistory
                    plot3(ax, obj.solverLog.history(i).xi(:,1), obj.solverLog.history(i).xi(:,2), obj.solverLog.history(i).xi(:,3), 'color',cc(i,:));
                end

            else
                disp('error, valid dimension<=3 !');
            end
        end

    end
end