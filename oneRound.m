function [x_new, chi_set] = oneRound(x, Z,odometry,n_iterations)
  
  nmeas=size(Z,1);
  chi_set = [];
  x_new  = x
  kernel_threshold = 100
  for(j=1:n_iterations)
    j

    H=zeros(7,7);
    b=zeros(7,1);
    chi=0;

  for (i = 1:nmeas)
    
    
    [e,J,status]=errorAndJacobian(x_new, Z(i,:),odometry(i,:));
    chi+=e'*e;
    if (chi>kernel_threshold)
      e*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    endif
    H+=eye(7)*0.01;
    H+=J'*J;
    b+=J'*e;
    
  endfor

  dx=-H\b;
  x_new += dx';

  x_new(7)=wrapToPi(x_new(7));

  chi_set = [chi_set;chi];
  endfor
end
