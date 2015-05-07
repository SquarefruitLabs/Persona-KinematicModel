/*
Squarefruit Labs - Persona
*/

arm1 = 100;
arm2 = 80;
baseoff = 50;
elboff = 20;
resolution = 50;
off = baseoff + elboff;

x = 20;
y = 80;
z = 50;

//Distance from the origin to the point. 
//Verify the target is within bounds
totdist = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
echo("totdist",totdist);

//cylindrical coordinates
rad2 = sqrt(pow(x,2)+pow(y,2));
//height
hgt = z;
//Angle
phi = atan2(y,x);

//Corresponding angle the base needs to rotate
corrangl = atan2(off, rad2);
//Corrected radius to account for offsets
corrrad = sqrt(off*off+rad2*rad2) - rad2;
echo("corr",corrrad,rad2);

//Proper targets in the robot's frame
targetx = x - corrrad*cos(phi);
targety = y - corrrad*sin(phi);
targetz = z;

//Corrected radius for the targets
rad = sqrt(targetx*targetx + targety*targety);


echo("targ", targetx, targety, targetz);
echo("xyz", x,y,z);

cyldist = sqrt(pow(rad,2)+pow(hgt,2));
echo("cyldist", cyldist);

//S,Q,E - pseudo SCARA arm parameters
kyu = acos((pow(rad,2)+pow(hgt,2)+pow(arm1,2)-pow(arm2,2))/(2*arm1*sqrt(pow(rad,2)+pow(hgt,2))));
eee = acos((pow(rad,2)+pow(hgt,2)-pow(arm1,2)-pow(arm2,2))/(2*arm1*arm2));
ess = -atan2(hgt, rad) - kyu;

echo("SE", ess, eee);

//Upper arm, assuming the ball is the shoulder
module arm_one()
{
	translate([0,baseoff,0])hull()
	{
		sphere(r=8,$fn = resolution);
		translate([arm1,0,0]) sphere(r=8, $fn=resolution);
	}
}

//Fore arm
module arm_two()
{
	translate([0,baseoff + elboff,0]) hull()
	{
		sphere(r=8, $fn=resolution);
		translate([arm2,0,0]) sphere(r=8, $fn= resolution);
	}
	translate([arm2, baseoff + elboff, 0]) rotate([0,-ess,0]) rotate([0,-eee,0])endEff();
}

translate([0,0,30])color("cyan",0.6)
{
rotate([0,0,phi - corrangl]) rotate([0,ess,0]) arm_one();
rotate([0,0,phi - corrangl]) rotate([0,ess,0]) translate([arm1, 0,0]) rotate([0,eee,0]) arm_two();
}

//Testing purposes only, to mark the final position.
module vector()
{
	hull()
	{
		sphere(r=2);
		translate([x,y,z]) sphere(r=2);
	}
}
//color("red") vector();

module endEff()
{
	
	hull()
	{
		sphere(r=8);
		translate([0,0,-15]) sphere(r=2);
	}

}


//translate([targetx, targety, targetz]) endEff();
module calc_rotation(x,y,arm_num)
{
	rot = acos((pow(x,2)+pow(y,2)+pow(arm1,2)-pow(arm2,2))/(2*arm1*sqrt(pow(x,2)+pow(y,2))));
	echo(arm_num, rot);
}

//End effector
module base()
{
	difference()
	{
	minkowski()
	{
	sphere(r = 35, center = true);
	sphere(r = 5, center = true);
	}
	translate([0,0,-70])cube([80,80,80], center = true);
	}
}
color("cyan",0.6)translate([0,0,30])base();

module actarm1()
{
	hull()
	{
	sphere(r = 2, center = true);
	translate([arm1, baseoff + elboff/2, 0]) sphere(r = 2, center = true);
	}
}

module actarm2()
{
	hull()
	{
		translate([0, baseoff + elboff/2, 0])sphere(r = 2, center = true);
		translate([arm2, baseoff + elboff, 0]) sphere(r=2, center=true);
	}
}

//rotate([0,0,phi]) rotate([0,ess,0]) actarm1();
//rotate([0,0,phi]) rotate([0,ess,0]) translate([arm1, 0,0]) rotate([0,eee,0]) actarm2();
