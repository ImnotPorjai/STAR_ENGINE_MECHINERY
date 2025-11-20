# STAR_ENGINE_MECHINERY
In this simulation, we will animate a star-engine mechanism which have 5 pistons and track the motion of its pistons as the central pentagon crank rotates around center. This exercise is based on concepts from mechanism design and kinematics, focusing on multi-degree-of-freedom linkages and their motion analysis. We will use MATLAB with SYMBOLIC MATH tool's box to construct the mechanism, scale the visual representation , and real time simulate the dynamic motion of the pistons and links.

# What our code can do.
1.Construct the star engine mechanism using crank–link vector loop equations.

2.Solve for the angular positions and piston coordinates as a function of crank angle.

3.Track the trajectory of piston 1 and optionally pentagon center.

4.Plot the radial velocity of piston 1 over one full rotation of the crank.

5.Display the piston stroke and highlight maximum and minimum positions.

When completed, the simulation will be:

![](Untitled(2).gif)









# Star Engine Simulation – Step-by-Step 
[🚀 Run the code](star_engine_v7_real.m)
Step 1: Load geometric parameters 
---> Define number of pistons, crank radius, link length, polygon size, piston width/height, link width, base crank speed.

Step 2: Define symbolic variables
---> Create theta_sym as a symbolic variable representing the crank angle.
---> Create arrays Px_sym and Py_sym to store piston x and y positions symbolically.

Step 3: Solve piston positions symbolically
---> Use vector loop equations for each piston:

Step 4: Compute radial velocity for piston 1
---> Use symbolic derivatives 

Step 5: Compute piston stroke
---> Stroke = max(Px) - min(Px)

Step 6: Prepare figures
---> Animation figure: shows crank pin, pentagon, pistons, links, central triangle 
---> Graph figure: shows piston radial velocity vs time and crank angular velocity vs time

Step 7: Main animation loop
---> Loop at each dt timestep: Update crank angle: theta_current += speed*dt
     Evaluate Px_current, Py_current from symbolic equations
     Update positions of pistons, links, pentagon, triangle
     Evaluate radial velocity numerically: v_piston_radial_plot
     Update plots, max/min velocity, crank speed

Step 8: Parameter function
---> engine_params() returns mechanism dimensions and base crank speed

Step 9: Pentagon vertices function
---> pentagon_coords(size) returns x,y coordinates for pentagon vertices for animation

