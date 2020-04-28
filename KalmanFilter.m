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
        
        function obj = OneStep(obj)
           D_x_ext = obj.F * obj.D_x * obj.F' + obj.G * obj.D_ksi * obj.G';
           obj.M = obj.H * D_x_ext * obj.H' + obj.D_n; 
           obj.K = D_x_ext * obj.H' * inv(obj.M);  
           obj.D_x = D_x_ext - obj.K * obj.H * D_x_ext;
           obj.res = obj.y - obj.y_ext;
%            obj.L = exp(-0.5*obj.discr'*obj.M^(-1)*obj.discr)/(2*pi*det(obj.M))^(4/2); % вычисление правдоподобия фильтра
           obj.X = obj.X_ext + obj.K*obj.res; % оценка вектора состояния
        end
        
    end
    
    methods (Abstract)
        
        obj = MakeMatrix(obj)
        obj = Update(obj)
        
    end
    
    
end

