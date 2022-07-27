classdef Box2d < handle
    
    properties
        center_
        length_ = 0.0
        width_ = 0.0
        half_length_ = 0.0
        half_width_ = 0.0
        heading_ = 0.0
        cos_heading_ = 0.0
        sin_heading_ = 0.0
        
        corners_ = []
        max_x_ = -inf
        min_x_ = inf
        max_y_ = -inf
        min_y_ = inf
        
        kEpsilon = 1e-10
    end
    
    methods
        function obj = Box2d(x, y, heading, length, width)
            obj.center_.x = x;
            obj.center_.y = y;
            obj.heading_ = heading;
            obj.length_ = length;
            obj.width_ = width;
            obj.half_length_ = length / 2.0;
            obj.half_width_ = width / 2.0;
            obj.cos_heading_ = cos(heading);
            obj.sin_heading_ = sin(heading);
            obj.InitCorners();
        end
        
        function out = IsPointIn(obj, x, y)
            x0 = x - obj.center_.x;
            y0 = y - obj.center_.y;
            dx = abs(x0 * obj.cos_heading_ + y0 * obj.sin_heading_);
            dy = abs(-x0 * obj.sin_heading_ + y0 * obj.cos_heading_);
            out = dx <= obj.half_length_ + obj.kEpsilon && dy <= obj.half_width_ + obj.kEpsilon;
        end
        
        function out = HasOverlap(obj, box)
            if box.max_x() < obj.min_x() || ...
                    box.min_x() > obj.max_x() || ...
                    box.max_y() < obj.min_y() || ...
                    box.min_y() > obj.max_y()
                out = false;
                return;
            end
            
            shift_x = box.center_x() - obj.center_.x;
            shift_y = box.center_y() - obj.center_.y;
            
            dx1 = obj.cos_heading_ * obj.half_length_;
            dy1 = obj.sin_heading_ * obj.half_length_;
            dx2 = obj.sin_heading_ * obj.half_width_;
            dy2 = -obj.cos_heading_ * obj.half_width_;
            
            dx3 = box.cos_heading() * box.half_length();
            dy3 = box.sin_heading() * box.half_length();
            dx4 = box.sin_heading() * box.half_width();
            dy4 = -box.cos_heading() * box.half_width();
            
            out = abs(shift_x * obj.cos_heading_ + shift_y * obj.sin_heading_) <= ...
                abs(dx3 * obj.cos_heading_ + dy3 * obj.sin_heading_) + ...
                abs(dx4 * obj.cos_heading_ + dy4 * obj.sin_heading_) + obj.half_length_ && ...
                abs(shift_x * obj.sin_heading_ - shift_y * obj.cos_heading_) <= ...
                abs(dx3 * obj.sin_heading_ - dy3 * obj.cos_heading_) + ...
                abs(dx4 * obj.sin_heading_ - dy4 * obj.cos_heading_) + obj.half_width_ && ...
                abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <= ...
                abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) + ...
                abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) + box.half_length() && ...
                abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <= ...
                abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) + ...
                abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) + box.half_width();
        end
        
        function obj = Shift(obj, x, y)
            obj.center_.x = obj.center_.x + x;
            obj.center_.y = obj.center_.y + y;
            obj.InitCorners();
        end
        
        function obj = InitCorners(obj)
            dx1 = obj.cos_heading_ * obj.half_length_;
            dy1 = obj.sin_heading_ * obj.half_length_;
            dx2 = obj.sin_heading_ * obj.half_width_;
            dy2 = -obj.cos_heading_ * obj.half_width_;
            
            corner1.x = obj.center_.x + dx1 + dx2;
            corner1.y = obj.center_.y + dy1 + dy2;
            corner2.x = obj.center_.x + dx1 - dx2;
            corner2.y = obj.center_.y + dy1 - dy2;
            corner3.x = obj.center_.x - dx1 - dx2;
            corner3.y = obj.center_.y - dy1 - dy2;
            corner4.x = obj.center_.x - dx1 + dx2;
            corner4.y = obj.center_.y - dy1 + dy2;
            obj.corners_ = [corner1, corner2, corner3, corner4];
            
            obj.max_x_ = max([corner1.x, corner2.x, corner3.x, corner4.x]);
            obj.min_x_ = min([corner1.x, corner2.x, corner3.x, corner4.x]);
            obj.max_y_ = max([corner1.y, corner2.y, corner3.y, corner4.y]);
            obj.min_y_ = min([corner1.y, corner2.y, corner3.y, corner4.y]);
        end
        
        function center_x = center_x(obj)
            center_x = obj.center_.x;
        end
        
        function center_y = center_y(obj)
            center_y = obj.center_.y;
        end
        
        function length = length(obj)
            length = obj.length_;
        end
        
        function width = width(obj)
            width = obj.width_;
        end
        
        function heading = heading(obj)
            heading = obj.heading_;
        end
        
        function half_length = half_length(obj)
            half_length = obj.half_length_;
        end
        
        function half_width = half_width(obj)
            half_width = obj.half_width_;
        end
        
        function cos_heading = cos_heading(obj)
            cos_heading = obj.cos_heading_;
        end
        
        function sin_heading = sin_heading(obj)
            sin_heading = obj.sin_heading_;
        end
        
        function corners = corners(obj)
            corners = obj.corners_;
        end
        
        function max_x = max_x(obj)
            max_x = obj.max_x_;
        end
        
        function min_x = min_x(obj)
            min_x = obj.min_x_;
        end
        
        function max_y = max_y(obj)
            max_y = obj.max_y_;
        end
        
        function min_y = min_y(obj)
            min_y = obj.min_y_;
        end
    end
end

