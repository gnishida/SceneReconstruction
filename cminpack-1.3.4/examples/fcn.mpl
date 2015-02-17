with(VectorCalculus):
fvec := Vector(15);
y := Vector([.14, .18, .22, .25, .29, .32, .35, .39, .37, .58, .73, .96, 1.34, 2.1, 4.39]);
for i to 15 do
  tmp1 := (i -1)+ 1:
  tmp2 := 15 - (i-1):
  if (i-1) > 7 then
    tmp3 := tmp2
  else
    tmp3 := tmp1
  fi:
  fvec[i] := y[i] -  (x[1] + tmp1/(x[2]*tmp2 + x[3]*tmp3)):
od:
Jacobian(fvec,[x[1],x[2],x[3]]);

# now, with the second variable, x[2] bounded between a=0.1 and b=0.9
a:=0.1:
b:=