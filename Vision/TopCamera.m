%NOTE: ALREADY INCLUDED IN PYTHON CODE.

function [ x_ball, y_ball, z_ball ] = TopCamera( x1, y1, a1, x2, y2, a2 )
% Reads input from two cameras in pixels, provides ball position in world 
% coordinate system in SI coordinates; assumes no distortion; assumes
% cameras are centered; assume we are given x_pic1, y_pic1, crop_x_top, 
% xrop_x_bottom crop_y_left, crop_y_right; assume we can calculate f_2 from
% analyzing reference images; assume we are given position of camera 2 in
% world coorinates, x_cam2, y_cam2, z_cam2; assume we are given x_pic2 and
% that "flat" (constant world z) is in the middle of the camera

r_x1 = (10 / 3.28084) / (x_pic1 - crop_x_top - crop_x_bottom);
r_y1 = (8 / 3.28084) / (y_pic1 - crop_y_left - crop_y_right);
x_ball = (x1 - crop_x_top) * r_x1;
y_ball = (y1 - crop_y_left) * r_y1;

d_xy = sqrt((x_ball-x_cam2)^2+(y_ball-y_cam2)^2);
z_ball = z_cam2 - (x2-(x_pic2)/2) / f_2 * d_xy;

end
