## Running the solution
- Use a similar setup to `./check_setup.py`, from the root of the repository running
  `ros2 launch hw01_sim simulation.launch.py preset:=1`.
- In another terminal, run `./solution_hw01.py` from `assignments/hw01/my_solution/`.

## Detecting the marker
We chose to use opencv for the marker detection.
Firstly we apply a color-mask, using `cv2.inRange`, and filter out any non-red colors.
Then, we follow [1], using `cv2.moments` to find the center of the resulting blob.

## Transforming from 2d coordinates in image frame into 3d camera coordinates frame
We're given the intrinsic matrix[2].
K = [[f_x,   0, c_x]
     [  0, f_y, c_y]
     [  0,   0,   1]]
As explained in [2], given a vector a = [u v w]^T in the camera coordinate frame,
let K v = [x y w]^T, the pixel (x, y) represents the corresponding pixel to the vector.

Therefore, to go from a' = [x y 1]^T to b = [u v 1] we need to invert K, we get
K^-1 = [[1/f_x,     0, -c_x/f_x]
        [    0, 1/f_y, -c_y/f_y]
        [    0,     0,        1]]

Now, notice that b = K^-1 [x y 1] is normalized, therefore for it to represent the marker position,
we need to multiply it by the depth z. We get that
- u = z * (x - c_x) / f_x
- v = z * (y - c_y) / f_y
And that is the representation of the pixel (x, y) in the camera coordinate frame.

## Technical Commentary about the code

## Results
We passed all 3 given presets,
with each reporting that we held for '2 simulated seconds', and 'live contact streams clear'.
We get the following tip errors:
- For preset 1, a tip error of 0.0060 m.
- For preset 2, we get a tip error 0.0038 m.
- lastly, for preset 3, a tip error of 0.0050 m.

## Sources
- [1]: <https://stackoverflow.com/a/12608954>
- [2]: <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html>
