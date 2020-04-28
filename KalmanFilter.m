classdef KalmanFilter
    
    
    properties 
        X; % State vector
        D_x; % Covariance
        
        F; % classic self-dynamic matrix for linear model; dF/dx for non-linear model
        G; % matrix of white gaussian noises impact; can depend on X
        D_ksi; % white gaussian noises; usually const
        H; % classic measurement matrix for linear model; dS/dx for non-linear model
        D_n; % measurement noises (WGN)
        
        y; % vector of measurements
        X_ext; % extrapolation of state vector; F*X for linear model; F(X) for non-linear model
        y_ext; % extrapolation of measurements; H*X_ext for linear model; S(X_ext) for non-linear model
        
        posts; % Reference points positions
    end
    
    methods
    end
    
end

