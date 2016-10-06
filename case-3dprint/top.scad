include<variables.scad>

// Inside Parts
module top_parts() color("red") {
    // No STL for NFC -> Cube
    #translate([battery_width+nozzle+wall_inside+nozzle/2,use_y/2-nfc_length/2,wall_outside]) cube([nfc_width,nfc_length,nfc_height]);
    // No STL for BLE -> Cube
    #translate([battery_width+nozzle+wall_inside+(use_x-battery_width-nozzle-wall_inside)/2,use_y/2,wall_outside+nfc_height]) cube(size=[bluetooth_length,bluetooth_width,bluetooth_height],center=true);
    // No STL for Battery -> Cube
    #translate([nozzle/2,use_y/2-battery_length/2,wall_outside]) cube([battery_width,battery_length,battery_height]);
}

module top() {
    //top plate
    difference() {
        // align to usable inner area
        union() translate([wall_outside_radius/2,wall_outside_radius/2,0]){
            // Top plate
            difference() {
                union() {
                    cube(size=[use_x_top-wall_outside_radius,use_y-wall_outside_radius,2*wall_outside_radius]);
                    // Cylinders for the rounded edges
                    translate([0,0,wall_outside_radius]) rotate([-90,0,0]) cylinder(h=use_y-wall_outside_radius,r=wall_outside_radius);
                    translate([use_x_top-wall_outside_radius,0,wall_outside_radius]) rotate([-90,0,0]) cylinder(h=use_y-wall_outside_radius,r=wall_outside_radius);
                    translate([0,0,wall_outside_radius]) rotate([0,90,0]) cylinder(h=use_x_top-wall_outside_radius,r=wall_outside_radius);
                    translate([0,use_y-wall_outside_radius,wall_outside_radius]) rotate([0,90,0]) cylinder(h=use_x_top-wall_outside_radius,r=wall_outside_radius);
                    // Spheres for the rounded corners
                    translate([0,0,wall_outside_radius]) sphere(wall_outside_radius);
                    translate([0,use_y-wall_outside_radius,wall_outside_radius]) sphere(wall_outside_radius);
                    translate([use_x_top-wall_outside_radius,0,wall_outside_radius]) sphere(wall_outside_radius);
                    translate([use_x_top-wall_outside_radius,use_y-wall_outside_radius,wall_outside_radius]) sphere(wall_outside_radius);
                }
                translate([-2*wall_outside_radius,-2*wall_outside_radius,wall_outside_radius]) cube(size=[use_x_top+3*wall_outside_radius,use_y+3*wall_outside_radius,2*wall_outside_radius]);
            }
            //outside walls
            translate([-wall_outside_radius,0,wall_outside_radius]) cube(size=[use_x+wall_outside_radius,use_y-wall_outside_radius,top_wall_height]);
            translate([0,-wall_outside_radius,wall_outside_radius]) cube(size=[use_x-wall_outside_radius,use_y+wall_outside_radius,top_wall_height]);
            translate([0,0,wall_outside_radius]) cylinder(h=top_wall_height,r=wall_outside_radius);
            translate([use_x-wall_outside_radius,0,wall_outside_radius]) cylinder(h=top_wall_height,r=wall_outside_radius);
            translate([0,use_y-wall_outside_radius,wall_outside_radius]) cylinder(h=top_wall_height,r=wall_outside_radius);
            translate([use_x-wall_outside_radius,use_y-wall_outside_radius,wall_outside_radius]) cylinder(h=top_wall_height,r=wall_outside_radius);
            }
        //place for battery
        translate([0,use_y/2-(battery_length/2+nozzle/2),wall_outside]) cube(size=[battery_width+nozzle,battery_length+nozzle,3*top_wall_height]);
        //place for NFC
        translate([battery_width+nozzle+wall_inside,use_y/2-(nfc_length+nozzle)/2,wall_outside]) cube(size=[nfc_width+nozzle,nfc_length+nozzle,3*top_wall_height]);
        translate([battery_width+nozzle+wall_inside,-wall_outside_radius/2+wall_outside/2,bottom_wall_height+wall_outside_radius-top_wall_height-layer-0.5]) cube(size=[use_x-battery_width-nozzle-wall_inside-1,use_y+wall_outside,top_wall_height-layer+0.5]);
    }
}
top();
//top_parts();
