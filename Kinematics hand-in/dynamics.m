%% Torques

L1 = Link('d', 0.352, 'a', 0.070, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.360, 'alpha', 0, 'offset', pi/2);
L3 = Link('d', 0, 'a', 0.445, 'alpha', 0, 'offset', -pi/2);

% Set the masses
L1.m = 40;
L2.m = 35;
L3.m = 25;

% Set center of gravity in the middle, has to be negative for some reason
L2.r = [-L2.a/2 0 0];
L3.r = [-L3.a/2 0 0];

irb = SerialLink([L1 L2 L3], 'name', 'IRB140');

q = deg2rad([0 10 40]);

% Specify gravity to be down, i.e. negative in z.
irb.gravload(q, [0 0 -9.81])
%% Trajectory

q0 = deg2rad([10 20 30]);
qf = deg2rad([-10 40 10]);

Q = jtraj(q0, qf, 100);

irb.plot(Q)

%% Free-fall

q = [0 0 0];
qd0 = [0 0 0];

[T, Q, Qd] = irb.fdyn(2, [], q0, q0);

irb.plot(Q)