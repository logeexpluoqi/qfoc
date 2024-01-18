clear; clc; close all;

T_32 = 2/3 * [1, -1/2, -1/2;
              0, sqrt(3)/2, -sqrt(3)/2;
              sqrt(2)/2, sqrt(2)/2, sqrt(2)/2];

t = 0:0.01:30;
amp = 5;

ia = amp * sin(t);
ib = amp * sin(t + 2 * pi/3);
ic = amp * sin(t - 2 * pi/3);

theta = atan2((sqrt(3)/2 * ib - sqrt(3)/2 * ic), (ia - 1/2 * ib - 1/2 * ic));

figure(1)
plot(t, ia, t, ib, t, ic);
legend("ia", "ib", "ic");
title("ia, ib, ic");
grid minor;

[ialpha, ibeta] = clarke(ia, ib, ic);
figure(2)
plot(t, ialpha, t, ibeta);
legend("ialpha", "ibeta");
title("clark transfer");
grid minor;

[id, iq] = park(ialpha, ibeta, theta);
figure(3)
plot(t, id, t, iq, t, theta, "-.");
legend("id", "iq", "theta");
title("park transfer");
grid minor;

[ualpha, ubeta] = ipark(id, iq, theta);
figure(4)
plot(t, ualpha, t, ubeta, t, theta, "-.");
legend("ud", "uq", "theta");
title("inverse park transfer");
grid minor;

[a, b, c] = iclark(ualpha, ubeta);
figure(5)
plot(t, a, t, b, t, c);
legend("ia", "ib", "ic");
title("inverse clark transfer");
grid minor;

function [ialpha, ibeta] = clarke(ia, ib, ic)
    ialpha = (2/3) * (ia - 1/2 * ib - 1/2 * ic);
    ibeta = (2/3) * (sqrt(3)/2 * ib - sqrt(3)/2 * ic);
end

function [ia, ib, ic] = iclark(ialpha, ibeta)
    ia = ialpha;
    ib = -(1/2) * ialpha + sqrt(3)/2 * ibeta;
    ic = -(1/2) * ialpha - sqrt(3)/2 * ibeta;
end

% d axis align to alpha axis
function [id, iq] = park(ialpha, ibeta, theta)
    id = ialpha .* cos(theta) + ibeta .* sin(theta);
    iq = -ialpha .* sin(theta) + ibeta .* cos(theta);
end

function [ualpha, ubeta] = ipark(ud, uq, theta)
    ualpha = ud .* cos(theta) - uq .* sin(theta);
    ubeta = ud .* sin(theta) + uq .* cos(theta);
end
