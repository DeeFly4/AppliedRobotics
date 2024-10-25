%% Torques

L1 = Link('d', 0.352, 'a', 0.070, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.360, 'alpha', 0, 'offset', pi/2);
L3 = Link('d', 0, 'a', 0.445, 'alpha', 0, 'offset', -pi/2);

L1.m = 40;
L2.m = 35;
L3.m = 25;

irb = SerialLink([L1 L2 L3], 'name', 'IRB140');

q = deg2rad([0 10 40]);

link2 = irb.A(2, q).t;
link23 = irb.A([2 3], q).t;

torque2 = L2.m/2 * link2(1)/1000 + L3.m/2 * link23(1)/1000;

link3 = link23 - link2;

torque3 = L3.m/2 * link3(1)/1000;

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