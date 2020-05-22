function [phi_nl, v_nl] = non_linear_block(phi,v)

K_phi = 0.3;
K_v = 2.5;

x =-(pi/2)+0.01:0.01:pi/2-0.01;
r =  -(pi/2.5)/min(x);
xr = x*r;
y = tan(x)*K_phi;


[~,map_idx]=min(abs(xr-phi));
phi_nl=y(map_idx);


r =  (7)/max(x);

y = tan(x)*K_v;
xr = x*r;
xr = [0 xr(ceil(end/2)+1: end)];
yr = [0 y(ceil(end/2)+1: end)];

[~,map_idx]=min(abs(xr-v));
v_nl=yr(map_idx);

end

