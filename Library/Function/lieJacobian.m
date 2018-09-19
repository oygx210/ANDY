function output=lieJacobian(x,h,f)
    jac=jacobian(h,x);
    output=jac*f;
end