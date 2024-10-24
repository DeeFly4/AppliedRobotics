L1 = Link('d', 352, 'a', 70, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 360, 'alpha', 0, 'offset', pi/2);
L3 = Link('d', 0, 'a', 445, 'alpha', 0, 'offset', -pi/2);

irb = SerialLink([L1 L2 L3], 'name', 'IRB140');

m1 = 40;
m2 = 35;
m3 = 25;

q = deg2rad([0 10 40]);

link2 = irb.A(2, q).t;
link23 = irb.A([2 3], q).t;

torque2 = m2/2 * link2(1)/1000 + m3/2 * link23(1)/1000;

link3 = link23 - link2;

torque3 = m3/2 * link3(1)/1000;