classdef ToFFilter < KalmanFilter
    % 2D navigation with known altitude
    % State Vector is X = (x; Vx; y; Vy);
    properties
        T; 
        c;
        h;
    end
    
    methods
        
        function obj = ToFFilter(X_initial, config) % constructor
            obj = obj.Init(X_initial);
            obj.posts = config.posts;
            obj.std_n = config.sigma_n;
            obj.std_ksi = config.sigma_a;
            obj.D_ksi = eye(2)*obj.std_ksi^2;
            obj.c = config.c;
            obj.h = config.hei;
        end
        
        function obj = MakeMatrix(obj,T)
            index = find(obj.y); 
            N = length(index);
            obj.D_n = obj.std_n^2*eye(N); % матрица шумов наблюдений
            obj.y_ext = zeros(N,1); % матрица связи наблюдений с вектором состояния
            obj.H = zeros(N,4); % производная матрицы S по вектору состояния
            dT = T - obj.T;
            obj.T = T;
            obj.F = [1 dT 0 0;
                     0 1 0 0;
                     0 0 1 dT;
                     0 0 0 1;];
            obj.X_ext = obj.F * obj.X;
            obj.G = [0 0; dT 0; 0 0; 0 dT];
            
            x  = obj.X_ext(1);
            y  = obj.X_ext(3);
            
            for i = 1:N
                obj.y_ext(index(i),1) = norm(obj.posts(:,index(i)) - [x;y;obj.h]);
            end
            
            k = 0;
            for i = 1:N
                k = k + 1;
                H(k,1) = (x - obj.posts(1,index(i)))/(R(index(i)));
                H(k,2) = 0;
                H(k,3) = (y - obj.posts(2,index(i)))/(R(index(i)));
                H(k,4) = 0;
            end
            
        end
        
        function obj = Update(meas,T)
            obj.y = meas;
            obj = obj.MakeMatrix(T);
            obj = obj.OneStep;
        end
            
        
        
    end
    
end

