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

function out = transition(id_from, id_to, v)
  out.id_from = id_from;
  out.id_to = id_to;
  out.v = v;
end
