include<variables.scad>
// Unterteil
use<bottom.scad>
// Oberteil
use <top.scad>

bottom();
//bottom_parts();
rotate([180,0,0]) translate([0,-use_y,-bottom_wall_height-top_wall_height-2*wall_outside_radius]) top();
//rotate([180,0,0]) translate([0,-use_y,-bottom_wall_height-top_wall_height-2*wall_outside_radius]) top_parts();