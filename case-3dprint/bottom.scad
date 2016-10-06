include<variables.scad>

// Inside Parts
module bottom_parts() color("red") {
// STL-Files from Sparkfun, seem to be a little too small, fits better in reality
// STL-Files not included because of unknown License, please download them yourself from Sparkfun and enable this module at the end of this file, if you want to see, how the modules are aligned in the case.
// https://www.sparkfun.com/products/10217    
#translate([charger_width-0.3,0,wall_outside+charger_stl_offset_z]) rotate([0,0,90]) import("SparkFun_Lipo_Charger_Basic-microUSB.stl");
// https://www.sparkfun.com/products/11114
#translate([charger_width+nozzle+wall_inside+0.7,0.7,wall_outside+arduino_stl_offset_z]) import("Arduino-Pro-Mini.stl");
}

module microusb() {
    translate([usbradius,wall_outside_radius,usbradius]) rotate([90,0,0]) union() {
        cylinder(h=2*wall_outside_radius,r=usbradius);
        translate([usbwidth-2*usbradius,0,0]) cylinder(h=2*wall_outside_radius,r=usbradius);
        translate([0,-usbradius,0]) cube(size=[usbwidth-2*usbradius,2*usbradius,2*wall_outside_radius]);
        translate([-usbradius/2,0,0]) cube(size=[usbwidth-usbradius,2*usbradius,2*wall_outside_radius]);
    }
}

module bottom() {
    difference() {
        // align to usable inner area
        union() translate([wall_outside_radius/2,wall_outside_radius/2,0]){
            // Bottom plate
            cube(size=[use_x-wall_outside_radius,use_y-wall_outside_radius,2*wall_outside+bottom_wall_height]);
            // Cylinders for the rounded edges
            translate([0,0,wall_outside_radius]) rotate([-90,0,0]) cylinder(h=use_y-wall_outside_radius,r=wall_outside_radius);
            translate([use_x-wall_outside_radius,0,wall_outside_radius]) rotate([-90,0,0]) cylinder(h=use_y-wall_outside_radius,r=wall_outside_radius);
            translate([0,0,wall_outside_radius]) rotate([0,90,0]) cylinder(h=use_x-wall_outside_radius,r=wall_outside_radius);
            translate([0,use_y-wall_outside_radius,wall_outside_radius]) rotate([0,90,0]) cylinder(h=use_x-wall_outside_radius,r=wall_outside_radius);
            // Spheres for the rounded corners
            translate([0,0,wall_outside_radius]) sphere(wall_outside_radius);
            translate([0,use_y-wall_outside_radius,wall_outside_radius]) sphere(wall_outside_radius);
            translate([use_x-wall_outside_radius,0,wall_outside_radius]) sphere(wall_outside_radius);
            translate([use_x-wall_outside_radius,use_y-wall_outside_radius,wall_outside_radius]) sphere(wall_outside_radius);
            //outside walls
            translate([-wall_outside_radius,0,wall_outside_radius]) cube(size=[use_x+wall_outside_radius,use_y-wall_outside_radius,bottom_wall_height]);
            translate([0,-wall_outside_radius,wall_outside_radius]) cube(size=[use_x-wall_outside_radius,use_y+wall_outside_radius,bottom_wall_height]);
            translate([0,0,wall_outside_radius]) cylinder(h=bottom_wall_height,r=wall_outside_radius);
            translate([use_x-wall_outside_radius,0,wall_outside_radius]) cylinder(h=bottom_wall_height,r=wall_outside_radius);
            translate([0,use_y-wall_outside_radius,wall_outside_radius]) cylinder(h=bottom_wall_height,r=wall_outside_radius);
            translate([use_x-wall_outside_radius,use_y-wall_outside_radius,wall_outside_radius]) cylinder(h=bottom_wall_height,r=wall_outside_radius);
            // alignment
            translate([battery_width+nozzle-wall_outside_radius/2+wall_inside+nozzle/2,-wall_outside_radius+wall_outside/2,bottom_wall_height+wall_outside_radius]) cube(size=[use_x-battery_width-nozzle-wall_inside-nozzle-1,wall_outside,top_wall_height-layer-0.5]);
            translate([battery_width+nozzle-wall_outside_radius/2+wall_inside+nozzle/2,-wall_outside_radius+wall_outside/2+use_y,bottom_wall_height+wall_outside_radius]) cube(size=[use_x-battery_width-nozzle-wall_inside-nozzle-1,wall_outside,top_wall_height-layer-0.5]);
            //front wall up to BLE
            translate([use_x-wall_outside_radius/2-wall_outside,-wall_outside_radius/2+use_y/2-(nfc_length-nozzle)/2,wall_outside_radius+bottom_wall_height]) cube(size=[2*wall_outside,nfc_length-nozzle,top_wall_height-(nfc_height-wall_outside)-0.5]);
        }
        // place for charger
        translate([0,0,wall_outside]) cube(size=[charger_width+nozzle,charger_length+nozzle,3*bottom_wall_height]);
        // place for arduino
        translate([charger_width+nozzle+wall_inside,0,wall_outside]) cube(size=[arduino_width+nozzle,arduino_length+nozzle,3*bottom_wall_height]);
        // inner wall not too high
        translate([charger_width,0,2*wall_outside+bottom_wall_height-1]) cube(size=[2*wall_inside,charger_length+nozzle,2]);
        // USB
        translate([(charger_width+nozzle)/2-usbwidth/2,0,wall_outside+usbdistance]) microusb();
    }
}
bottom();
//bottom_parts();
