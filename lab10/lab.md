## Objective

The goal of this lab was to implement grid localization using a Bayes Filter in simulation. The robot does not know its true position, so it must estimate its position using noisy motion (odometry) and sensor measurements. The objective is to show that probabilistic localization can track the robot better than raw odometry.

---

## Code Implementation

The localization code was divided into five main functions: compute_control(), odom_motion_model(), prediction_step(), sensor_model(), and update_step(). Each one matches one part of the Bayes Filter.

#### compute_control()

This function calculates the control input between two poses. Given the previous pose and current pose, it finds the first rotation needed to face the direction of motion, the translation distance, and the second rotation needed to match the final direction.

```cpp
def compute_control(cur_pose, prev_pose):
    cur_x, cur_y, cur_theta = cur_pose
    prev_x, prev_y, prev_theta = prev_pose

    delta_x = cur_x - prev_x
    delta_y = cur_y - prev_y

    delta_rot_1 = degrees(atan2(delta_y, delta_x)) - prev_theta
    delta_rot_1 = loc.mapper.normalize_angle(delta_rot_1)

    delta_trans = dist((cur_x, cur_y), (prev_x, prev_y))

    delta_rot_2 = cur_theta - prev_theta - delta_rot_1
    delta_rot_2 = loc.mapper.normalize_angle(delta_rot_2)

    return delta_rot_1, delta_trans, delta_rot_2
```

#### odom_motion_model()

This function computes the probability of moving from one state to another given the measured control input u. First, the function computes the motion actually required to go from prev_pose to cur_pose. Then it compares that motion to the measured odometry input u. Gaussian distributions are used for the two rotations and the translation. The final transition probability is the product of those three probabilities.

```cpp
def odom_motion_model(cur_pose, prev_pose, u):
    actual_rot_1, actual_trans, actual_rot_2 = compute_control(cur_pose, prev_pose)

    prob_rot_1 = loc.gaussian(actual_rot_1 - u[0], 0, loc.odom_rot_sigma)
    prob_trans = loc.gaussian(actual_trans - u[1], 0, loc.odom_trans_sigma)
    prob_rot_2 = loc.gaussian(actual_rot_2 - u[2], 0, loc.odom_rot_sigma)

    prob = prob_rot_1 * prob_trans * prob_rot_2
    return prob
```

#### prediction_step()

This function performs the prediction step of the Bayes Filter. It updates loc.bel_bar, which is the predicted belief before using new sensor data. The function first gets the odometry control input from the previous and current odometry poses. Then, for every possible current grid cell, it sums contributions from every possible previous grid cell. Each contribution is previous belief at that state multiplied by the transition probability from motion model.

A small optimization was used. If a previous belief is less than 0.0001, it is skipped because it contributes very little but costs time to compute.

```cpp
def prediction_step(cur_odom, prev_odom):
    u = compute_control(cur_odom, prev_odom)

    loc.bel_bar = np.zeros(loc.bel.shape)

    for x_idx in range(loc.mapper.MAX_CELLS_X):
        for y_idx in range(loc.mapper.MAX_CELLS_Y):
            for a_idx in range(loc.mapper.MAX_CELLS_A):

                cur_pose = loc.mapper.from_map(x_idx, y_idx, a_idx)
                total = 0

                for prev_x in range(loc.mapper.MAX_CELLS_X):
                    for prev_y in range(loc.mapper.MAX_CELLS_Y):
                        for prev_a in range(loc.mapper.MAX_CELLS_A):

                            bel_prev = loc.bel[prev_x, prev_y, prev_a]

                            if bel_prev < 0.0001:
                                continue

                            prev_pose = loc.mapper.from_map(prev_x, prev_y, prev_a)
                            total += odom_motion_model(cur_pose, prev_pose, u) * bel_prev

                loc.bel_bar[x_idx, y_idx, a_idx] = total

    if np.sum(loc.bel_bar) > 0:
        loc.bel_bar = loc.bel_bar / np.sum(loc.bel_bar)
```

#### sensor_model()

This function computes the probability of a sensor observation for one grid cell. Here, obs is the expected set of 18 measurements for a given grid cell, and loc.obs_range_data.flatten() is the actual measured sensor data from the robot. The Gaussian gives the likelihood for each individual measurement. The result is an array of 18 probabilities.

```cpp
def sensor_model(obs):
    prob_array = loc.gaussian(obs, loc.obs_range_data.flatten(), loc.sensor_sigma)
    return prob_array
```

#### update_step()

This function performs the update step of the Bayes Filter. It uses the measured sensor data to correct the predicted belief. For each grid cell, the expected observation is generated using mapper.get_views(). The sensor model compares that expected observation with the actual robot measurement. Since the 18 range measurements are assumed independent, the probabilities are multiplied together using np.prod(prob_array). This is then multiplied by the predicted belief loc.bel_bar to get the updated belief loc.bel. Finally, the belief is normalized again.

```cpp
def update_step():
    for x_idx in range(loc.mapper.MAX_CELLS_X):
        for y_idx in range(loc.mapper.MAX_CELLS_Y):
            for a_idx in range(loc.mapper.MAX_CELLS_A):

                true_obs = loc.mapper.get_views(x_idx, y_idx, a_idx)
                prob_array = sensor_model(true_obs)

                loc.bel[x_idx, y_idx, a_idx] = np.prod(prob_array) * loc.bel_bar[x_idx, y_idx, a_idx]

    if np.sum(loc.bel) > 0:
        loc.bel = loc.bel / np.sum(loc.bel)
```

<br>

---

## Results

The Bayes Filter was tested on a defined trajectory in the simulator. The results show that the estimated belief follows the ground truth closely, while the odometry drifts over time and is noisy.

Video 1 and 2 below shows two trials of the run.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/KfRxgFy2JuE"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 1:</b> Trial 1.
</p>

<p align="center">
  <img src="../img/lab10/trial1.png" width="80%">
</p>
<p align="center">
  <b>Figure 1:</b> Trial 1 Final Plot.
</p>

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/3KCuoqqc-RI"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 2:</b> Trial 2.
</p>

<p align="center">
  <img src="../img/lab10/trial2.png" width="80%">
</p>
<p align="center">
  <b>Figure 2:</b> Trial 2 Final Plot.
</p>

---

## Most Probable States Comparison

The most probable state after each iteration of the Bayes filter was compared with the ground truth pose. From the results, the estimated belief state is generally very close to the ground truth in position, with errors typically within about 0.1 to 0.3 meters. After the update step, the belief often converges to a single grid cell with high probability (close to 1.0), showing that the sensor measurements are effective in correcting the prediction. Although the reported angle error can appear large, this is mainly due to angle wrapping (for example, 360° is equivalent to 0°), so the orientation estimate is still reasonable. Overall, the comparison shows that the Bayes filter is able to accurately track the robot’s position over time.

<p align="center">
  <img src="../img/lab10/table.png" width="80%">
</p>

| t | GT (x, y) | Belief (x, y) | Error (x, y) | Prob |
|---|----------|---------------|--------------|------|
| 0 | (0.287, -0.087) | (0.305, 0.000) | (-0.018, -0.087) | 1.0 |
| 1 | (0.520, -0.539) | (0.610, -0.610) | (-0.090, 0.071) | 1.0 |
| 2 | (0.520, -0.539) | (0.610, -0.610) | (-0.090, 0.071) | 1.0 |
| 3 | (0.548, -0.951) | (0.305, -0.914) | (0.244, -0.037) | 1.0 |
| 4 | (0.814, -1.091) | (0.914, -0.914) | (-0.101, -0.177) | 1.0 |
| 5 | (1.608, -0.915) | (1.524, -0.914) | (0.084, -0.001) | 1.0 |
| 6 | (1.682, -0.522) | (1.524, -0.610) | (0.158, 0.087) | 1.0 |
| 7 | (1.750, -0.162) | (1.829, -0.305) | (-0.079, 0.142) | 1.0 |
| 8 | (1.743, 0.337) | (1.829, 0.000) | (-0.086, 0.337) | 0.79 |
| 9 | (1.736, 0.673) | (1.829, 0.914) | (-0.093, -0.241) | 1.0 |
| 10 | (1.310, 0.935) | (1.219, 0.914) | (0.091, 0.020) | 1.0 |
| 11 | (0.421, 0.791) | (0.610, 0.914) | (-0.188, -0.124) | 1.0 |
| 12 | (0.296, 0.142) | (0.305, 0.000) | (-0.008, 0.142) | 1.0 |
| 13 | (0.072, -0.189) | (0.000, -0.305) | (0.072, 0.116) | 1.0 |
| 14 | (-0.292, -0.372) | (-0.305, -0.305) | (0.013, -0.067) | 1.0 |
| 15 | (-0.698, -0.395) | (-0.610, -0.305) | (-0.088, -0.091) | 0.75 |

---

## Discussion

This lab focused on using a Bayes Filter to estimate the robot's position. The robot uses odometry for motion, but it is noisy and causes error over time. The prediction step spreads the belief, while the update step uses sensor measurements to correct it. The two steps together make the estimate more accurate. Overall, the Bayes filter works well and is able to track the robot much better than odometry alone.

---

## Acknowledgment

I referenced [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab10/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
