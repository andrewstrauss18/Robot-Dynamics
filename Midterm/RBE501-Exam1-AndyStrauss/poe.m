% RBE 501 - Robot Dynamics - Sping 2022
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<Andy Strauss>***
clear, clc, close all
addpath('utils');

%% *** ENTER THE LAST DIGIT OF YOUR WPI ID BELOW: ***
digit = [0];

%% Create the manipulator
robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;
robot.teach(zeros(1,6))


%% Calculate the forward kinematics using the Product of Exponentials
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
H1 = .1348;
H2 = .2738;
H3 = .230;
H4 = .116;
Ht = H1+H2+H3+H4;
W1 = .1283;
W2 = .105;
Wt = W1+W2;

S1 = [0 0 1 0 0 0]';
S2 = [1 0 0 0 H1 0]';
S3 = [1 0 0 0 (H1+H2) 0]';
S4 = [1 0 0 0 (H1+H2+H3) 0]';
S5 = [0 0 1 0 -W1 0]';
S6 = [1 0 0 0 Ht 0]';

S = [S1 S2 S3 S4 S5 S6];

% Let us also calculate the homogeneous transformation matrix M for the
% home configuration
M = [ 0  0 1 Wt;
     -1  0 0  0;
      0 -1 0 Ht;
      0  0 0  1];