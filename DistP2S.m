% Calculate the minimum Distance between a Point to a Segment

function dist = DistP2S(x,a,b)
d_ab = norm(a-b);
d_ax = norm(a-x);
d_bx = norm(b-x);
if d_ab ~= 0 
    if dot(a-b,x-b)*dot(b-a,x-a)>=0
        A = [b-a;x-a];
        dist = abs(det(A))/d_ab; % Formula of point - line distance       
    else
        dist = min(d_ax, d_bx); 
    end
else % if a and b are identical
    dist = d_ax;
end