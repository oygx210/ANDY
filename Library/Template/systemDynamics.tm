function [TF,Vel,Cor,Jac,TransDis,MM,GFI,GFF,GFU,JacU,JacCons,CorCons,GFCons]=FUNCNAME_(t,q,p,u,s)
    
    [TF,Vel,Cor,Jac,TransDis]=KINEMATICS_(t,q,p,u,s);
    [MM,GFI]=INERTIAL_(t,q,p,u,s,TF,Vel,Cor,Jac);
    [GFF]=FORCE_(t,q,p,u,s,TF,TransDis,Vel,Jac);
    [GFU,JacU]=INPUT_(t,q,p,u,s,TF,TransDis,Vel,Jac);
    [JacCons,CorCons,GFCons]=CONSTRAINT_(t,q,p,u,s,TransDis,Vel);
