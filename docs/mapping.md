# Mapping

**Why is Mapping needed?**

The center of mass of the car at t = 0 is designated as the global frame. The map of the environment
is created with respect to this frame. The localization module, as discussed above, gives the position
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
