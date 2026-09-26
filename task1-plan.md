Aim

Fill in the ackermann_wheel_angles() function of the Task 1A boilerplate so that, given a single desired steering angle, it returns the two real front-wheel angles based on your vehicle’s Ackermann geometry.

The Function You Implement

ackermann_wheel_angles() is the only function you write. It receives one steering angle and returns the pair of wheel angles it implies:

```python
def ackermann_wheel_angles(delta):
    ...
    return left_wheel_angle, right_wheel_angle
```
Sign convention. A positive delta means the vehicle is turning left. Keep this consistent for both returned angles - it is what Task 1B’s controller will assume when it calls your function.

Adding Helper Functions

You are not expected to fit everything into one function. Add as many helper functions and global variables as you like - put them below ackermann_wheel_angles() but above the END OF YOUR IMPLEMENTATION line, and call them from ackermann_wheel_angles():

example:
def ackermann_wheel_angles(delta):
    ...
    turn_radius = radius_from_angle(delta)      # your helper, defined below
    ...
    return left_wheel_angle, right_wheel_angle


# ---- helpers, still inside the implementation block ----

def radius_from_angle(delta):
    ...
    return radius

keep in mind. List any helpers and globals you add in the file header at the top of the script.
