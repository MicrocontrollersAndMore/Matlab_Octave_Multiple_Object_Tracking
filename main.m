% main.m

% This code is 100% credit to the MathWorks organization, the makers of Matlab.  This is a reproducti%on of the code found at:
% 
% http://www.mathworks.com/help/vision/examples/motion-based-multiple-object-tracking.html
% 
% I've only made minor changes to make the MathWorks example to make it compatible with Octave 4.0.0.

close all;          % close and clear anything remaining from a previous run
clear all;

pkg load image;      % added this line for compatibility with Octave

multiObjectTracking();    % call the outer function in the other file

