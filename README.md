# CarND-Path-Planning-Project   

### Reflection of Path Planning

##### 1. Use of spline.h
* Simulator is providing the last points that havent been traversed by the simulator.I use the last two points and push it into two arrays 
```c
ptsx,ptsy
```
*	In case the lenght of the previous points are less than 2 I push cars current points and the previous points using the following equation.
```c
double prev_car_x = car_x - cos(car_yaw);
double prev_car_y = car_y - sin(car_yaw);
```
* Creating three new waypoints to generate continuous path.
```
getXY(car_s+30,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y)
getXY(car_s+60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y)
getXY(car_s+60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y)
```
<br> Here lane is current lane of the car.<br>

The points generated above were in global co-ordinates and will be converted to local car co-ordinates to make calculation easier.

```c
/*double ref_yaw = deg2rad(car_yaw)
or 
ref_yaw =  atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
*/
double shiftx = ptsx[i] - ref_x;
double shifty = ptsy[i] - ref_y;

ptsx[i] = shiftx*cos(-ref_yaw)-shifty*sin(-ref_yaw);
ptsy[i] = shiftx*sin(-ref_yaw)+shifty*cos(-ref_yaw);
```

* fitting these points using spline.
* pushing all the remaining waypoints from previous path to `next_x_vals` and `next_y_vals`.
* creating new points based on velocity.
```c
target_dist = 30;
double N = (target_dist/(.02*velocity/2.24)); 
```
* These new points will be added to `next_x_vals` and `next_y_vals` in addition to previous points.

#### 2.Cars Behaviour
* For smooth transition of car from steady state to moving 
	* Velocity at intial =0;
	* Incrementing velocity if no vehicle ahead on the current lane at certain distance.
```c
//49 max velocity.
if(velocity<49){
	velocity += 0.5;
}
```
* For vehicles that are there within the collision range on the same lane and lane switching is not possible.
```
velocity-=0.4;
```
decreasing velocity to avoid collision.


#### 3. Behaviour of other cars using sensor fusion.
* Simulator is passing x and y components of velocity of other cars,Using this data to check if our trajectory is overlapping with any other trajectory in same lane. If that is the case then,reducing velocity.
* A flag `too_close` for all cars ahead of our car and setting `behind` flag for cars behind our car
* These flags will be used for lane change.

* `too_close` flag for each lane suggest which lane is following conditins for lane switching based on if the other car predicted s is greater that current car s and the difference is larger that 40.
```c
//check_car_s = other_car_s + previous_path_size*0.02*other_car_speed;
 if(check_car_s>car_s && (check_car_s-car_s)<40)
{
too_close[j] = true;
}
```

* `behind` flag for each lane suggest which lane is following conditions for lane switching based on if the other car predicted s is less than current car s and the difference is greater than 15.

```c
//check_car_s = other_car_s + previous_path_size*0.02*other_car_speed;
 if(check_car_s < car_s && (car_s-check_car_s)<15)
{
behind[j] = true;
}
```
#### 4. Lane change
* Checking if a car in our current lane is slowing down or its trajectory is colliding with ours.
	* Chosing next lane based on current current lane+-1 
	* next lane must not have a vehicle whose trajectory is colliding with ours ie it must be sufficient distance ahead of us while lane changing and must not speed-up behind us while lane changing.
	* we set the lane to the lane number that follow these conditions and in later step we generate trajectory points based on that next lane and let spline generate the trajectory for smooth transition.

Following code Changes the lane:-
```c
if(too_close[lane])
          {
            cout<<"Someone on my lane"<<endl;
            if(lane == 0)
            {
              if(!too_close[1] && !behind[1])
              {
                lane = 1;
              }
            }
            else if(lane == 1)
            {
              if(!too_close[0] && !behind[0])
              {
                lane = 0;
              }
              else
              {
                if(!too_close[2] && !behind[2])
                {
                  lane = 2;
                }
              }
            }
            else{
              if(!too_close[1] && !behind[1])
              {
                lane = 1;
              }
            }
            velocity -= 0.4;
          }
          else
          {
            if(velocity<49)
            {
              cout<<"keep Accelarating"<<endl;
              velocity+=0.4;
            }
           
          }
```
