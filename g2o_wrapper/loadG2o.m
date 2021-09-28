% #   This source code is part of the localization and SLAM package
% #   deveoped for the lectures of probabilistic robotics at 
% #   Sapienza, University of Rome.
% #  
% #     Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti
% #  
% #   It is licences under the Common Creative License,
% #   Attribution-NonCommercial-ShareAlike 3.0
% #  
% #   You are free:
% #     - to Share - to copy, distribute and transmit the work
% #     - to Remix - to adapt the work
% #  
% #   Under the following conditions:
% #  
% #     - Attribution. You must attribute the work in the manner specified
% #       by the author or licensor (but not in any way that suggests that
% #       they endorse you or your use of the work).
% #    
% #     - Noncommercial. You may not use this work for commercial purposes.
% #    
% #     - Share Alike. If you alter, transform, or build upon this work,
% #       you may distribute the resulting work only under the same or
% #       similar license to this one.
% #  
% #   Any of the above conditions can be waived if you get permission
% #   from the copyright holder.  Nothing in this license impairs or
% #   restricts the author's moral rights.
% #  
% #   This software is distributed in the hope that it will be useful,
% #   but WITHOUT ANY WARRANTY; without even the implied 
% #   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% #   PURPOSE.
% #
%   
% # load a file.g2o file and returns the four structs of landmark, poses, transitions, observations

function [landmarks, poses, transitions, observations] = loadG2o(filepath)

	%%-----G2O specification---
	VERTEX_XY = 'VERTEX_XY';
	VERTEX_SE2 = 'VERTEX_SE2';
	ROBOTLASER1 = 'ROBOTLASER1';
	EDGE_SE2 = 'EDGE_SE2';
	EDGE_BEARING_SE2_XY = 'EDGE_BEARING_SE2_XY';
	EDGE_SE2_XY = 'EDGE_SE2_XY';
	%%-------------------------

	%open the file
	fid = fopen(filepath, 'r');
	

	%debug stuff
	i_vert_xy = 0;
	i_vert_se2 = 0;
	i_robotlaser = 0;
	i_edge_se2 = 0;
	i_edge_se2_xy = 0;
	i_edge_bearing_se2_xy=0;
        poses = struct();
        landmarks = struct();
        transitions = struct();
        observations = struct();
	%    
	curr_id = -1;

	while true
		%get current line
		c_line = fgetl(fid);

		%stop if EOF
		if c_line == -1
			break;
		end

		%Split the line using space as separator
		elements = strsplit(c_line,' ');

		switch(elements{1})
			case VERTEX_XY
        			landmarks(end+1) = extractLandmark(elements);
				i_vert_xy = i_vert_xy + 1; %do not use pre/post increment. Keep the Matlab compatibility
			case VERTEX_SE2
        			poses(end+1) = extractPose(elements);
				i_vert_se2 = i_vert_se2 + 1;
			case ROBOTLASER1
			        %TODO
				i_robotlaser = i_robotlaser + 1;
			case EDGE_SE2
			        transitions(end+1) = extractTransition(elements);
				i_edge_se2 = i_edge_se2 + 1;
			case EDGE_BEARING_SE2_XY
				current_obs = extractBearing(elements);
				if current_obs.pose_id == curr_id
					observations(end).observation(end+1) = current_obs.observation;
				else
				        observations(end+1) = current_obs;
					curr_id = observations(end).pose_id;
					i_edge_bearing_se2_xy = i_edge_bearing_se2_xy + 1;
				end
			case EDGE_SE2_XY
				current_obs = extractPoint(elements);				
				if current_obs.pose_id == curr_id
					observations(end).observation(end+1) = current_obs.observation;
				else
				        observations(end+1) = current_obs;
					curr_id = observations(end).pose_id;
					i_edge_se2_xy = i_edge_se2_xy + 1;
				end

			otherwise
				%disp('Error in reading first element');
		end
	end
  
  printf('\n|--> landmarks: %d \n|--> poses: %d \n',i_vert_xy, i_vert_se2);
  printf('|--> transitions: %d \n|--> observations: %d \n',i_edge_se2, i_edge_bearing_se2_xy); 
  fflush(stdout);
end

function out = extractLandmark(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  out = landmark(id,[x_pose,y_pose]);
end

function out = extractPose(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  th_pose = str2double(elements{5});
  out = pose(id,x_pose, y_pose, th_pose);
end

function out = extractTransition(elements)
  from_id = str2double(elements{2});
  to_id = str2double(elements{3});
  x_t = str2double(elements{4});
  y_t = str2double(elements{5});
  th_t = str2double(elements{6});
  out = transition(from_id,to_id, [x_t;y_t;th_t]);
end

function out = extractBearing(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  bearing = str2double(elements{4});
  out = observation(from_id,land_id, bearing);
end

function out = extractPoint(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  x_p = str2double(elements{4});
  y_p = str2double(elements{5});
  out = observation(from_id,land_id, [x_p; y_p]);
end
