function [deg,rad] = dms2deg(deg,mins,secs)
% CONVDEGMINSSECS: converts degrees, mins, secs to decimal degrees and
% output answer in decimal degrees and radians
% INPUTS: deg,mins,secs
% OUTPUTS: deg,rad
%
% EXAMPLE of USAGE:
% convdegminssecs(45,54,55)
% 
% ans = 45.9153
%
deg = deg + mins/60 + secs/3600;
rad = deg * pi/180;
end