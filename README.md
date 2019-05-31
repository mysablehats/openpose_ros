# openpose_ros

This package is a simplified implementation of an openpose wrapper. We developed
it before we had knowledge of other implementations such as https://github.com/stevenjj/openpose_ros/tree/master/openpose_ros_pkg
which also mentions https://github.com/firephinx/openpose_ros and  https://github.com/tue-robotics/openpose_ros which are probably more well kept
and user friendly. We kept using our implementation since it is already working
with docker and it does not try to be too clever and give 3d positions of the
joints (since we also read from a dataset that does not have depth).

We could have decided to ditch this implementation and just modify others so as to
publish zeros in depth, or even try to estimate depth with an additional neural
network, but this was considered to take more time and the idea was abandoned.

Possibly in the future this is to be replaced by one of those implementations and shelved.
