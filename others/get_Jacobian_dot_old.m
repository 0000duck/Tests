function J_dot = get_Jacobian_dot(q,dq)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
dq4 = dq(4);
dq5 = dq(5);
dq6 = dq(6);
dq7 = dq(7);

J_dot1 = [  sin(q1)*(cos(q2)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*dq2*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250)) - cos(q1)*((33*cos(q3)*dq3)/400 - sin(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q1)*(cos(q2)*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250))*dq1 + sin(q1)*dq1*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            sin(q1)*(cos(q2)*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250))*dq1 - cos(q1)*(cos(q2)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*dq2*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250)) - sin(q1)*((33*cos(q3)*dq3)/400 - sin(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*dq1*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            0];

J_dot2 = [  cos(q1)*(sin(q2)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250)) - sin(q1)*(sin(q2)*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250))*dq1; ...
            sin(q1)*(sin(q2)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250)) + cos(q1)*(sin(q2)*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250))*dq1; ...
            sin(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*dq2*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + 79/250)];

J_dot3 = [  sin(q1)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q1)*dq1*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*cos(q2)*((33*cos(q3)*dq3)/400 - sin(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*sin(q1)*dq1*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q1)*sin(q2)*dq2*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            sin(q1)*dq1*(cos(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - (33*cos(q3))/400 + sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*((33*sin(q3)*dq3)/400 + cos(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*sin(q1)*((33*cos(q3)*dq3)/400 - sin(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*cos(q2)*dq1*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q1)*sin(q2)*dq2*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            - sin(q2)*((33*cos(q3)*dq3)/400 - sin(q3)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q3)*dq3*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*dq2*((33*sin(q3))/400 - sin(q3)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))];

J_dot4 = [  cos(q1)*(sin(q2)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*cos(q3)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*sin(q3)*dq3*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000))) - sin(q1)*sin(q3)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q1)*(sin(q2)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*cos(q3)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)))*dq1 + cos(q1)*sin(q3)*dq1*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q1)*dq3*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            sin(q1)*(sin(q2)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*dq2*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*cos(q3)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q3)*sin(q2)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q2)*sin(q3)*dq3*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000))) + cos(q1)*sin(q3)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q1)*(sin(q2)*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*cos(q3)*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)))*dq1 + sin(q1)*sin(q3)*dq1*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*cos(q3)*dq3*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)); ...
            cos(q3)*sin(q2)*((33*cos(q4)*dq4)/400 - cos(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + sin(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q5)*sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q4)*cos(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + sin(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*(sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - (33*sin(q4)*dq4)/400 + cos(q4)*dq4*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*dq2*((33*cos(q4))/400 + sin(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - cos(q4)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*sin(q3)*dq3*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q2)*cos(q3)*dq2*(cos(q4)*((11*sin(q6))/125 - (107*cos(q6))/1000 + 48/125) - (33*sin(q4))/400 + cos(q5)*sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000))];

J_dot5 = [  sin(q1)*(cos(q5)*sin(q3)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q3)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*cos(q4)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000)) + cos(q1)*(sin(q2)*(cos(q5)*sin(q3)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq2 - cos(q2)*(cos(q5)*sin(q3)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*cos(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q3)*sin(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*sin(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q2)*sin(q4)*sin(q5)*dq2*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q2)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q2)*sin(q4)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*(cos(q3)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq1 + sin(q1)*(cos(q2)*(cos(q5)*sin(q3)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*sin(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq1; ...
            sin(q1)*(sin(q2)*(cos(q5)*sin(q3)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq2 - cos(q2)*(cos(q5)*sin(q3)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*cos(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q3)*sin(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000)) + sin(q2)*sin(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q2)*sin(q4)*sin(q5)*dq2*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q2)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q2)*sin(q4)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000)) - cos(q1)*(cos(q5)*sin(q3)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q3)*cos(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*sin(q3)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*cos(q4)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*sin(q3)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q3)*sin(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q1)*(cos(q3)*cos(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*sin(q3)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq1 - cos(q1)*(cos(q2)*(cos(q5)*sin(q3)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000)) - sin(q2)*sin(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq1; ...
            sin(q2)*sin(q4)*sin(q5)*dq2*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q2)*(cos(q5)*sin(q3)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((11*cos(q6))/125 + (107*sin(q6))/1000))*dq2 - cos(q2)*sin(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - cos(q2)*cos(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q2)*cos(q5)*sin(q4)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - sin(q2)*(cos(q5)*sin(q3)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q3)*cos(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*sin(q5)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) - sin(q3)*sin(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q3)*cos(q4)*cos(q5)*dq5*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*sin(q3)*sin(q5)*dq3*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q3)*sin(q4)*sin(q5)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000))];
 
J_dot6 = [  sin(q1)*(sin(q3)*(sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q5)*sin(q4)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) + cos(q3)*sin(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq3 - cos(q3)*cos(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125) + sin(q3)*sin(q5)*dq3*((107*cos(q6))/1000 - (11*sin(q6))/125)) + cos(q1)*(sin(q2)*(sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - cos(q4)*cos(q5)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + sin(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q2)*(cos(q3)*(sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q5)*sin(q4)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - sin(q3)*sin(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - sin(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq3 + cos(q3)*sin(q5)*dq3*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q5)*sin(q3)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q2)*(cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2 - cos(q2)*(cos(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q4)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2) + sin(q1)*(cos(q2)*(cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q2)*(cos(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q4)*((107*cos(q6))/1000 - (11*sin(q6))/125)))*dq1 + cos(q1)*(sin(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq1; ...
            sin(q1)*(sin(q2)*(sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - cos(q4)*cos(q5)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + sin(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q2)*(cos(q3)*(sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q5)*sin(q4)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - sin(q3)*sin(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - sin(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq3 + cos(q3)*sin(q5)*dq3*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q5)*sin(q3)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q2)*(cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2 - cos(q2)*(cos(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q4)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2) - cos(q1)*(sin(q3)*(sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q5)*sin(q4)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) + cos(q3)*sin(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq3 - cos(q3)*cos(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125) + sin(q3)*sin(q5)*dq3*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q1)*(cos(q2)*(cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q2)*(cos(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q4)*((107*cos(q6))/1000 - (11*sin(q6))/125)))*dq1 + sin(q1)*(sin(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq1; ...
            - cos(q2)*(sin(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q5)*sin(q4)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - cos(q4)*cos(q5)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + sin(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - sin(q2)*(cos(q3)*(sin(q4)*((107*cos(q6)*dq6)/1000 - (11*sin(q6)*dq6)/125) + cos(q4)*dq4*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q4)*cos(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) + cos(q5)*sin(q4)*dq4*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q4)*sin(q5)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - sin(q3)*sin(q5)*((11*cos(q6)*dq6)/125 + (107*sin(q6)*dq6)/1000) - sin(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq3 + cos(q3)*sin(q5)*dq3*((107*cos(q6))/1000 - (11*sin(q6))/125) + cos(q5)*sin(q3)*dq5*((107*cos(q6))/1000 - (11*sin(q6))/125)) - cos(q2)*(cos(q3)*(sin(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) - cos(q4)*cos(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125)) + sin(q3)*sin(q5)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2 - sin(q2)*(cos(q4)*((11*cos(q6))/125 + (107*sin(q6))/1000) + cos(q5)*sin(q4)*((107*cos(q6))/1000 - (11*sin(q6))/125))*dq2];
 
 

J_dot7 = [0; 0; 0];


J_dot = [J_dot1, J_dot2, J_dot3, J_dot4, J_dot5, J_dot6, J_dot7];

end
