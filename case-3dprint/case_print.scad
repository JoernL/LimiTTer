include<variables.scad>
// Unterteil
use<bottom.scad>
// Oberteil
use <top.scad>
translate([0,wall_outside_radius,0]) bottom();
translate([0,-use_y-wall_outside_radius,0]) top();