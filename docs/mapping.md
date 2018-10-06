# Mapping

## Why is Mapping needed?

The center of mass of the car at t = 0 is designated as the global frame. The map of the environment
is created with respect to this frame. The localization module gives the position
of the center of mass of the car with respect to global frame at any time ‘t’. On the other hand, the
perception module gives the position of cones with respect to the image frame of the stereo camera.
The mapping module takes the cone list provided by perception module and uses the filtered
odometry given by localization to place the detected cones in the global frame. The resulting map is
used by Navigation to construct trajectory for car.

Now, in the first lap, the map is unknown. So the trajectory generation needs to be short-term, or
reactive to the cones detected immediately. Once the first lap is completed and the map is
constructed, the trajectory planning could be long-term or pro-active. These two scenarios translated
to a need for two different kinds of control strategies: reactive control and pro-active control.
Correspondingly, the mapping module creates two kinds of map at any time ‘t’ : a reactive map and
a global map. The reactive map is a filtered map of cones in the immediate field of view of the car.
On the contrary, the global map contains positions of all the cones ever seen by the car. Since global
map is used for long-term planning, the information inside it needs to be extremely robust. Hence
the flow of information is as follows:

Detected cones (Perception) → Reactive Map → Global Map

## reactive_mapping node

### Input
- /perception_cones : An expanding 1-D array received from perception, with size of multiples of 4. For every cone there is a data packet consisting of 4 values. Every packet has format [x, y, z, color] where x, y, z are 3D coordinates of a cone in camera frame. N is the number of cones detected in the image

### Output
- /reactive_cones : An expanding 1-D array with size of multiples of 4. For every cone there is a data packet consisting of 4 values. Every packet has format [centroid_x, centroid_y, variance, color], where first two values are centroid of cone position, the third value is the variance in its position and 4th value is its color. 

### Explanation
- Let's assume perception is detecting 2 cones every frame, then /perception_cones would send 2x4 array for every frame. The callback_perception() function integrates data from 3 frames (so, in this case, it'll collect 2x3 = 6 cone data), and calls cluster_frames() function. As name suggests, this function assigns a cluster id to every cone. If number of cones in a cluster is >= 65% (i.e 2), they are probably legitimate (have been seen 65% of the time) and their position is used to compute centroid (mean) and variance in cone positions. 

- It is observed that perception data is not noise free. In particular, some cones can change color abruptly. To tackle this, for a legitimate cluster, voting decides the actual color of that cone. Since we integrate 3 frames, the majority color is taken. If a cluster has only 2 cones, the color result might go wrong, but it's taken care of by global mapping :)

At the end, /reactive_cones topic is published as defined above. 

This method serves three purposes. The first one is to tackle the inherent defeciencies in perception object detection. The second one is to serve as a pre-filtering to global mapping. Lastly, the reactive map is used by navigation to carry reactive path planning and control, which could be useful in first lap (exploration).



## global_mapping node
### Input
- /reactive_cones : See previous section for definition
- /odometry/filtered: filtered odometry from robot localization package

### Output
- /global_map_markers : An expanding 1-D array in multiples of 7. For every cone there is a data packet consisting of 7 values. This packet has the following formnat: 

[x, y, color, covariance, hits, inFOV, id]

where x, y and color, are as usual, the coordinates, color and covariance of cone in global coordinates. The other parameters are described in detail below. 

### Explanation
In main, there are two subscribers, with a callback each for /reactive_cones and /odometry/filtered. In every callback, the message data is stored to its local class variable. In odometry callback, the rotation and translation matrix (R and T) are also computed. In main is also a timer, which calls a function to update global cone db at a fixed frequency. This was done to keep the update frequency as a design parameter, which can be changed as per discussion with navigation.

**Updating cone db**

Global map of cones has to be absolutely correct, because all the long term decisions of the car are based on this map. If a car sees a cone somewhere it shouldn't, it might divert towards it or stop pre-maturely. In worst case, it might crash into a cone which was perhaps missed. Reactive mapping provides a pre-filtering layer for entry to global database. But for global db, we need to go further. 

Our first thought was that a reactive cone should be added to global db only if it was seen repeatedly. But then the map creation would be slow and disjointed (cones appear and disappear without obvious reason). Also, navigation suggested keeping a covariance measure of position of cones. But in the covariance measure, we also wanted to encode the logic that if cone is seen many times, the covariance decreases. So we divided covariance measure into two parts: 

1. Every new cone in global db starts with max covariance radius. With time, if it's seen again, its 'hits' increase and proportionately, the covariance radius decreases. 
2. The cone position is updated using exponential moving average. Hence, every cone is characterized by its position (moving on an avg) and its covariance radius 

But that's not enough to remove spurious outliers that might creeep into global map (they always do). As Patric talked about in DD2410, mapping needs to have a temporary memory. Or in other words, a map should be able to forget. Hence, if a global cone (i.e. a cone in global db) is in field of view of car but is not seen frequently, then its hits reduce. Consequently, its covariance radius increases, till it's removed from global db. An analogy might be that every new cone starts off as a fat-ass baloon which loses air if it's seen frequently. But if it isn't seen anymore, then it inflates once more and 'pops' out of global map.  

Every cone is also given a unique id, which perhaps helps with data association.

With this background in mind, the update_cone_db() function has following logic:

- Firstly, only those reactive cones which are in field of view (FOV) of car are chosen (stored in reactive_in_fov_local).
- *reactive_in_fov_local* (list of cones in camera frame) is transformed to *reactive_in_fov_map* using R and T computed above.
- If cone_db is empty, add all *reactive_in_fov_map* to it.
- Else, for cones in *reactive_in_fov_map*, we do data association with cones in global db. In other words, for each cone in *reactive_in_fov_map*, try to find a cone in global db which is close enough (closeness is defined by self.R_max_cov). If you find a close enough cone in global database, that means it was seen again by the car. For this cone, add the hits, update its position (moving average) and covariance radius. The list of global cones which are successfully associated is stored in *db_in_reactive*. 
- All the cones in *reactive_in_fov_map* which were not associated with a cone in global db are new. So, these are added as new cones to global db. Every new cone is also assigned a unique id.
- Now, to encode the 'map forgetting' logic, we need to find all those cones in global db, which are within field of view (FOV) of car, but were not associated with any reactive cone. To do this, we transform the cone db to local coordinates (again, using R and T but the other way this time). Then we first find those cones which are within FOV circle of car (stored in *db_local_in_distance_index*). Then we find a subset in *db_local_in_distance_index* which is within FOV angle of car (stored in *db_local_in_angle_index*). Lastly, those cones which are in *db_local_in_angle_index*, but not *db_in_reactive* are the ones for which hits are subtracted. If hits hit a minimum, those cones are removed from global db.  

