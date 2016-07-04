%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2016/6/28 Yoshi R supported by Tokuma I @ Univercity of Tokyo
% The class files for RLS estimation.
% Explanation about this algorithm in Japanese is here
%  ( URL : http://yoh.ise.ibaraki.ac.jp/edu/graduateschool/Project1.pdf )
% 6/29 Updated # add property forgetting_function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef rls_const < handle
% Estimate Constant value 
    properties (SetAccess = protected)
        % Estimated Value
        Theta = 0
        % Matrix
        Pn = zeros(1);
        % Forgetting factor
        Rho = 0.99;
        % Function for update forgetting factor
        forgetting_factor_updater = @(x)  (1 - 0.01) * x + 0.01;
    end
    properties (SetAccess = public)
        % None
    end
    
    methods
        
        % constructer
        function obj = rls_const(n,Rho0)
           obj.Theta = zeros(2*n,1);
           % Define alpha = 1000;
           obj.Pn = 1000 * eye(2*n);
           % forgetting factor
           obj.Rho = Rho0;
        end
        
        % reinitialize in hand code
        function reinitialize(obj,Theta0,P0,Rho0)
           obj.Theta = Theta0;
           obj.Pn = P0;
           obj.Rho = Rho0;
        end
        
        % set forgetting factor updater 
        function set_FF_Updater(obj,f)
             if isa(f,'function_handle')
                 obj.forgetting_factor_updater = f;
             else
                 error('Please set function_handle !!')
             end
        end
        
        function set_Theta0(obj,Theta0)
            obj.Theta = Theta0;
        end
        
        % Updating
        function Estimated = estimate(obj,Yn,Zn)
            % Updating forgetting factor
            obj.Rho = obj.forgetting_factor_updater(obj.Rho);
            % Prepare for caluclate 
            Num = obj.Rho + (Zn.') * obj.Pn * Zn;
            Ln = obj.Pn * Zn / Num;
            En = Yn - Zn.' * obj.Theta;
            % Update Pn
            obj.Pn = 1/ obj.Rho * ( obj.Pn - (obj.Pn * Zn * (Zn.') * obj.Pn / Num) );
            % Update Estimation
            obj.Theta = obj.Theta + Ln * En;
            % Output
            Estimated = obj.Theta;
        end
        
    end
    
end