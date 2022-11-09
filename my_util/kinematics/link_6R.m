classdef link_6R < handle
    %link_6R: simple open chain 6R robot kinematics using body twist(Lie group representation)
    %             *note: just use with coppeliasim
    %   jointScrewAxis:body twist;  tipPose:[x;y;z;qx;qy;qz;qw]
    
    properties
        jointScrewAxis;
        jointAngles;
%         tipPose;
        tipSE3_init;
        tipSE3;
       % spatial_Jacob;
        body_Jacob;
    end
    
    methods
        function obj = link_6R(jointScrewAxis, tipSE3)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.jointScrewAxis = jointScrewAxis;
            obj.tipSE3_init = tipSE3;  
            obj.tipSE3 = tipSE3;
        end
        
        function [tipSE3, Jacob_b] = updateFK(obj, jointAngles)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.jointAngles = jointAngles;
            Jacob_b = zeros(6,6);
            AdTrans = SE3.exp(zeros(6,1));
            TransFK = SE3.exp(zeros(6,1));
            for i=6:-1:1
                if i==6
                    Jacob_b(:, i) = obj.jointScrewAxis(:, i);
                else
                    Jacob_b(:, i) = AdTrans.Ad() * obj.jointScrewAxis(:, i);
                end
                
                TransFK = TransFK * SE3.exp(obj.jointAngles(7-i) * obj.jointScrewAxis(:, 7-i));
                AdTrans = AdTrans * SE3.exp(-obj.jointAngles(i) * obj.jointScrewAxis(:, i)); %update SE3 trans
            end
            
            obj.body_Jacob = Jacob_b;
            obj.tipSE3 = obj.tipSE3_init * TransFK;
            tipSE3 = obj.tipSE3;
        end
    end
end

