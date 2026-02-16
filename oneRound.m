function [x_new, chi_set] = oneRound(x, Z,odometry)
  
  nmeas=size(Z,1);
  chi_set = [];
  x_new  = x

  for(j=1:100)
    j

    H=zeros(7,7);
    b=zeros(7,1);
    chi=0;

  for (i = 1:nmeas)
    if(Z(i,2)==0)
      continue
    endif
    
    [e,J]=errorAndJacobian(x_new, Z(i,:),odometry(i,:));
    H+=J'*J;
    b+=J'*e;
    chi+=e'*e;
  endfor

  dx=-H\b;
  x_new += dx';

  x_new(7)=wrapToPi(x_new(7));

  chi_set = [chi_set;chi];
  endfor
end
