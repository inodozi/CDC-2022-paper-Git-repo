function output=AA(A_f,A_g,B_f,B_g)
A_fg=A_f-A_g;
B_fg=B_f-B_g;
Z=zeros(2,1);
output=[A_fg B_fg Z B_g];
end