classdef KalmanFilter
    
    
    properties 
        X; % State vector
        D_x; % Covariance
        
        F; % classic self-dynamic matrix for linear model; dF/dx for non-linear model
        G; % matrix of white gaussian noises impact; can depend on X
        D_ksi; % white gaussian noises; usually const
        H; % classic measurement matrix for linear model; dS/dx for non-linear model
        D_n; % matrix of measurement noises (WGN)
        K; % gain
        M; % extrapolation of errors
        L; % likelihood (for adaptation)
        res; % residual
        
        y; % vector of measurements
        X_ext; % extrapolation of state vector; F*X for linear model; F(X) for non-linear model
        y_ext; % extrapolation of measurements; H*X_ext for linear model; S(X_ext) for non-linear model
        
        std_n; % standard deviation of measurenment noises;
        std_ksi; % standard deviation of process noises;
        
        posts; % Reference points positions
    end
    
    methods
        
        function obj = Init(obj,X_initial)
           obj.X = X_initial;
           obj.D_x = eye(length(X_initial));
        end
        
%         function obj = Init(obj,X_initial)
%            obj.X = X_initial;
%            obj.D_x = eye(length(X_initial));
%         end
        
    end
    
    methods (Abstract)
        
        obj = OneStep(obj)
        
    end
    
    
end

