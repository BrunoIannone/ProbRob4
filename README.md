# Calibration of a (real) Robot

## Task Overview

The dataset contains data collected from the sensors of a **real mobile robot**. Namely, a **front-tractor tricycle**.

The **output** should be:

### 1. Sensor Position

- **2D position of the sensor** with respect to the **base link**

### 2. Kinematic Parameters

- **`ksteer`**: radians per tick (of the steering encoder)
- **`ktraction`**: meters per tick (of the traction encoder)
- **`steer_offset`**: steering angle corresponding to zero wheel position
- **`base_line`**: length of the baseline. **Remember that the kinematic center is in the middle of the axis of the rear wheels**

---

## Dataset File

### `dataset.txt`

The first 10 lines contain:

- **Kinematic model**: traction_drive_wheel (front-tractor tricycle)
- **Kinematic parameters to be estimated**
- **Initial guesses** for the parameters
- **Encoder order** 
- **Encoder maximum ranges**
- **Laser w.r.t. base link transform** 

> [!IMPORTANT]
>
> The encoder order and ranges are critical for correctly interpreting the encoder tick data.

---

## Data Record Structure

Each record in the dataset is composed of the following fields:

### 1. Time

- **`time`**: Timestamp of the measurement

### 2. Encoder Ticks

- **`ticks`**: Encoder readings
    - **Steering encoder**: absolute encoder
    - **Traction encoder**: incremental encoder

### 3. Model Pose

- **`model pose`**: Odometry computed using the kinematic model

> [!NOTE]
>
> This model **does not correspond** to the one used in AMR.
>
> You are free to define and use your own kinematic model for this task. Hence, this field can safely be ignored if not needed.

### 4. Tracker Pose

- **`tracker pose`**: Position of the sensor obtained from an external odometry/tracking system

---

## Encoder Data Notes

> [!WARNING]
>
> Encoder readings are stored as **`uint32`** values, so **overflow may occur** in some cases.

> [!TIP]
>
> Detect and avoid overflow cases.
>
> Use **incremental differences** between encoder readings when integrating the kinematic model.

---

## Methodology

### State

* Qualify the domain

$$
x_r = \begin{pmatrix} k_s \\ k_t \\ so \\ b \end{pmatrix} \in \mathbb{R}^4, \quad
x_s = \begin{pmatrix} ^rx_s \\ ^ry_s \\ ^r\theta_s \end{pmatrix} \in SE(2)
$$

Therefore:

$$
\vec{x} = \begin{pmatrix} x_r \\ x_s \end{pmatrix} = \begin{pmatrix} k_s \\ k_t \\ so \\ b \\ ^rx_s \\ ^ry_s \\ ^r\theta_s \end{pmatrix} \in \mathbb{R}^4 \times SE(2)
$$

* Define an Euclidean parametrization for the perturbation

$$
\Delta x = \begin{pmatrix} \Delta x_r  \\ \Delta x_s \end{pmatrix} \in \mathbb{R}^7
$$

Where

$$
\Delta x_r = \begin{pmatrix}
\Delta k_s \\ \Delta k_t \\ \Delta so \\ \Delta b
\end{pmatrix} \in \mathbb{R}^4
$$

$$
\Delta x_s = \begin{pmatrix}
\Delta ^rx_s \\ \Delta ^ry_s \\ \Delta ^r\theta_s
\end{pmatrix} \in \mathbb{R}^3
$$

* Define  boxplus operator 

$$
^rX'_s = {^rX}_s \boxplus \Delta x_s = \text{v2t}(\Delta {^rx}_s) \cdot {^rX_s}
$$

### Measurements

* Qualify the domain

$$
\vec{z}_1 = \begin{pmatrix} t_s \\ t_t \end{pmatrix} \in \mathbb{R}^2
$$

$$
\vec{z}_2 = \begin{pmatrix} ^wx_s \\ ^wy_s \\ ^w\theta_s \end{pmatrix} \in SE(2)
$$

* Define an Euclidean parametrization for the perturbation
    * $\vec{z_1}$ is already Euclidean, so no boxminus needed
    * $\vec{z_2}$\vec{z}_2 = \begin{pmatrix} ^wx_s \\ ^wy_s \\ ^w\theta_s \end{pmatrix} \in SE(2) 

