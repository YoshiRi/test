function [Cta, Pn, Rn] = rls_const_func(Cta_,Pn_,Rn_,Yn ,Zn)
%% forgetting factor (Time varying)
Rn = (1 - 0.01)*Rn_ + 0.01;

%% function
Num = Rn+Zn'*Pn_*Zn;
Pn = 1/Rn*( Pn_ - (Pn_ * Zn * Zn' * Pn_/Num) );
Ln = Pn_*Zn /Num;
En = Yn - Zn.' * Cta_;

%% Output
Cta = Cta_ + Ln * En;

end