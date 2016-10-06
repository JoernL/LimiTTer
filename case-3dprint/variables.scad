$fa=0.2; // default minimum facet angle is now 0.5
$fs=0.2; // default minimum facet size is now 0.5 mm
// Variables
    // Arduino
        arduino_width=19;
        arduino_length=34;
        // With space for wires
        arduino_height=5;
        arduino_stl_offset_z=0.8;
    // Charger
        charger_width=11;
        charger_length=30;
        // With space for wires
        charger_height=5;
        charger_stl_offset_z=0.8;
	usbradius=3.5/2;
	usbwidth=9;
	usbdistance=1;
    // Battery
        battery_width=12.7;
        battery_length=31.4;
        battery_height=4.5;
    //Wireless
        nfc_width=38.5;
        nfc_length=28;
        nfc_height=2;
        bluetooth_length=14;
        bluetooth_width=18;
        bluetooth_height=2;
	wireless_height=nfc_height+bluetooth_height;
    // Printer
    nozzle=0.5;
    layer=0.2;
    // Walls
        wall_outside=1.5;
        wall_outside_radius=3;
        wall_inside=wall_outside;
        use_x=arduino_width+wall_inside+charger_width+2*nozzle;
        use_x_top=nfc_width+wall_inside+battery_width+2*nozzle;
        use_y=max(arduino_length,charger_length)+nozzle;
        bottom_wall_height=max(arduino_height,charger_height)-wall_outside+layer;
	top_wall_height=max(battery_height,wireless_height)-wall_outside+layer;
