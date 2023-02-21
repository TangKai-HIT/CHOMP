function [sampled_robot, namelist] = add_body_samplePts(robot, link_name, linkLen, num_samples)
%ADD_BODY_SAMPLEPTS add sample points on robot
%   robot: robot rigid body
%   link_name: names of the  body link to be sampled (cell array)
%   linkLen
%   num_samples: number of samples w.r.t each link_name (array )
%   sampled_robot
%   namelist: 

sampled_robot = copy(robot);
namelist = cell(1, length(link_name));

for i=1:length(link_name)
    len_sample = linkLen(i)/(num_samples(i));
    for j = 1: num_samples(i)
        %define sampled bodys
        namelist{i}{j} = [link_name{i}, sprintf('-sample%d', j)];
        sample_body = rigidBody(namelist{i}{j});
        tform = trvec2tform([j*len_sample, 0, 0]); % User defined
        setFixedTransform(sample_body.Joint, tform);
        addBody(sampled_robot, sample_body, link_name{i}); 
    end
end