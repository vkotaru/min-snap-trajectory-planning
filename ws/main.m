% planning for quadrotor with load suspended via 'n' link flexible cable.
% Initial and and final conditions specified.
%
% ---------------------------------------------------------------------
% Author: Prasath Kotaru (vkotaru@andrew.cmu.edu)
% Date: Oct-28-2016
% Last Updated: Nov-10-2016
% ======================================================================

%% ----------------------
% INITIALIZING WORKSPACE
%  ----------------------
% Clear workspace
% clear; 
close all; 
% clc;

% Add Paths
% Geometric Control Toolbox
addpath(genpath('fcns'));
addpath(genpath('../../GeoControl-Toolbox/')); % Add the location of your tool-box

%% -----------------------------
%   DEFINE PROBLEM
% ------------------------------

% Start and End points
QUAD.x0 = [0;0;0]; % Starting point
QUAD.xf = [1;0;10]; % End point

% Path properties
% ---------------
QUAD.params.n = 4; % No. of segments the path is divided into

% Polynomial degree type
%   same: Highest degree of the polynomial for all n segment is same
%   multiple: Different degree for each segment of the polynomial
QUAD.options.degtype = 'same';
QUAD.params.d = 4;


% Constraints
QUAD.params.rr = 2; % Degree of continuity to be maintained between the segments

% Cos Function: 
QUAD.params.r = 1; % Minimizing r^th order

% timings
%   [] - Total time is set as decision variable
%   10 - (numeric)[seconds] Total time to reach END from START
QUAD.time = [];
% timeframe
%   equal - equal time between different segments
%   diff - different time segments (to leave it upto optimization)
QUAD.options.timeframe = 'diff';


QUAD.options.tol = 1e-6;


%% ------------------------------
%   PLANNER OPTIONS
% -------------------------------







%% ------------------------------
%   PATH PLANNER
% -------------------------------

trajSoln = pathFinder(QUAD);


%% ------------------------------
%   PLOTS
% -------------------------------
% close all;
plotresults(trajSoln);


%%                                  *END*


