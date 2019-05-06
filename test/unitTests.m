% unitTests.m
% Tests individual modules
clearvars; close all; clc;
addpath('..\src');

% For now let's have a list of functions that we can comment or uncomment
% to run the tests.

% fluidUnitTests
results.fluidtest = fluidtest;
disp('Fluid test results:');
disp(results.fluidtest.overall);
% nodeUnitTests
% linkUnitTests
% tetherUnitTests

function r = fluidtest
    % make a fluid object
    % constructing without arguments makes still water
    fl = fluid;
    % speed should be zero
    if norm(fl.meanvelocity) > 0
        r.overall = 'fail: velocity';
        return
    end
    % flowtype should be still
    if fl.flowtype ~= flow.still
        r.overall = 'fail: flowtype';
        return
    end
    
    % Make sure we can change flowtype only to a flow enum
    try
        fl.flowtype = 5;
    catch ME
        disp(ME.message);
    end
    fl.flowtype = flow.linearZ;
    
    % Test the linearZ flowtype
    fl.meanvelocity = [1.0;0;0];
    point = [0;0;0];
    fl.computePointVelocity(point);
    disp(fl.pointvelocity);
    point = [1;1;25];
    fl.computePointVelocity(point);
    disp(fl.pointvelocity);
    
    r.overall = 'pass';
end % fluidtest
