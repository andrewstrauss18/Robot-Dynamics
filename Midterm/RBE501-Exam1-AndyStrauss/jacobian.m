% RBE 501 - Robot Dynamics - Spring 2022
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<Andy Strauss>***
clear, clc, close all
addpath('utils');

%% *** ENTER THE LAST DIGIT OF YOUR WPI ID BELOW: ***
poe

%% Create the manipulator
robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;

%% Calculate the Jacobian matrix in the home configuration
J = jacob0(S,zeros(1,6));