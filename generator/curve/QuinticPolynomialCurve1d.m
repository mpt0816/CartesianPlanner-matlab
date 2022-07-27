classdef QuinticPolynomialCurve1d < PolynomialCurve1d
    %UNTITLED3 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        coef_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        start_condition_ = [0.0, 0.0, 0.0]
        end_condition_ = [0.0, 0.0, 0.0]
    end
    
    methods
        function obj = QuinticPolynomialCurve1d(x0, dx0, ddx0, x1, dx1, ddx1, param)
            obj.ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
            obj.start_condition_(1) = x0;
            obj.start_condition_(2) = dx0;
            obj.start_condition_(3) = ddx0;
            obj.end_condition_(1) = x1;
            obj.end_condition_(2) = dx1;
            obj.end_condition_(3) = ddx1;
            obj.param_ = param;
        end
        
        function outputArg = Evaluate(obj, order, p)
            if p > obj.param_
                outputArg = obj.EvaluateLinearExtraplation(order, p);
            else
                switch(order)
                    case 0
                        outputArg = ((((obj.coef_(6) * p + obj.coef_(5)) * p + obj.coef_(4)) * p + obj.coef_(3)) * p + obj.coef_(2)) * p + obj.coef_(1);
                    case 1
                        outputArg = (((5.0 * obj.coef_(6) * p + 4.0 * obj.coef_(5)) * p + 3.0 * obj.coef_(4)) * p + 2.0 * obj.coef_(3)) * p + obj.coef_(2);
                    case 2
                        outputArg = (((20.0 * obj.coef_(6) * p + 12.0 * obj.coef_(5)) * p) + 6.0 * obj.coef_(4)) * p + 2.0 * obj.coef_(3);
                    case 3
                        outputArg = (60.0 * obj.coef_(6) * p + 24.0 * obj.coef_(5)) * p + 6.0 * obj.coef_(4);
                    case 4
                        outputArg = 120.0 * obj.coef_(6) * p + 24.0 * obj.coef_(5);
                    case 5
                        outputArg = 120.0 * obj.coef_(6);
                    otherwise
                        outputArg = 0.0;
                end
            end
        end
        
        function outputArg = ParamLength(obj)
            outputArg = obj.param_;
        end
        
        function outputArg = Coef(obj, order)
            outputArg = obj.coef_(order + 1);
        end
        
        function outputArg = Order()
            outputArg = 5;
        end
        
        function ComputeCoefficients(obj, x0, dx0, ddx0, x1, dx1, ddx1, p)
            obj.coef_(1) = x0;
            obj.coef_(2) = dx0;
            obj.coef_(3) = ddx0 / 2.0;
            
            p2 = p * p;
            p3 = p2 * p;
            
            c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
            c1 = (dx1 - ddx0 * p - dx0) / p2;
            c2 = (ddx1 - ddx0) / p;
            
            obj.coef_(4) = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
            obj.coef_(5) = (-15.0 * c0 + 7.0 * c1 - c2) / p;
            obj.coef_(6) = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
            
        end
        
        function outputArg = EvaluateLinearExtraplation(obj, order, p)
            s = obj.Evaluate(0, obj.param_);
            v = obj.Evaluate(1, obj.param_);
            a = obj.Evaluate(2, obj.param_);
            
            t = p - obj.param_;
            
            switch(order)
                case 0
                   outputArg = s + v * t + 0.5 * a * t * t;
                case 1 
                   outputArg = v + a * t;
                case 2
                   outputArg = a;
                otherwise
                   outputArg = 0.0;
            end
        end
    end
end

