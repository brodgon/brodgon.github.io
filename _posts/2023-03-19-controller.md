---
layout: post
title: Practice 1 - Follow-Line
subtitle: Week 2- Controlling F1’s Brain
social-share: false
readtime: true
---
Following the challenges proposed in [my previous post]( https://brodgon.github.io/2023-03-15-setting-up/), this week has been fully dedicated to solve them and improve the functioning of my robot.

As a reminder, we are facing, for the first time, the implementation of **PID reactive control robot** which is able to fulfill a racing circuit in the lowest time possible. For its development the [Unibotics platform](https://unibotics.org/) is used.

## The starting point: Where are we?

Last week, we shortly discussed the use of the simulator and introduce a first and really simple controller based on different scenarios. Additionally, prior to the implementation of the controller, we defined an image processing pipeline aimed to detect the red line the robot needs to follow.  All the procedure is schematized below.

![Last week’s pipeline](https://github.com/brodgon/brodgon.github.io/blob/master/docs/fig%20.png?raw=true)

As seen, the line detection relies on the white pixels’ count on a predefined row of the image. This row was selected after examining the image and considering that the red line always falls on its bottom half. 

Although the proposed method was proven to work, several weaknesses can be identified:

- Centroid detection may be affected by noisy pixels of the mask. When we binarize the input image, the thresholding might not be perfect, leaving some white residues outside the line. Since the detection method is based on the localization of the first and last white pixels appearing on a row, noisy pixels situated outside our contour may distort the centroid position, affecting the global result of the robot.

- Centroid detection will be disturbed by the presence of other red objects. Last week, we assumed that the red line was the only red object that appeared in our image; this, although is true for the selected simulation, is far from being realistic. In reality, other objects with similar tones may appear, like advertisement panels or people dressed in red. If any of these objects come on the scene, we won’t detect them or, if detected, they will affect the overall functioning of our robot.

- Line won’t be detected if it appears outside the selected row. If the eyes of the robots are moved to point in other direction, the line might fall outside the selected margin, which will make the computation of the centroid impossible.

These flaws may be solved in different ways: we can process the image deeper, causing the loop to be slower, or we can take advantage of image processing libraries. We will discuss the new processing method in next sections. 

When the image was processed, the modeled scene which adjust the best to the real scene was selected. The scenes were modeled depending on the angular velocity of the previous loop: if angular velocity was small, we let the robot to move faster (set a bigger linear speed); on the contrary, if angular velocity was high, the speed was set to low values.  This resulted on a really fluctuating robot which was able to complete a lap in 2:56 min. 

Although the lap was completed, the robot behavior did not satisfy the requirements: we need a faster robot which is not continuously zigzagging. The continuous fluctuations make the robot to be slower. Additionally, the scenarios model is quite weak, since we are not able to completely model the scene. 

In order to battle these drawbacks, we will set some new pipelines and discuss other types of controllers. 

## Processing: A new pipeline

To solve the problems outlined before, a new line detector was implemented. Just as the previous detector, the first step is to localize the line in the input image. To do so, we will reuse the **HSV color filter** defined last week. HSV filters will make our detector robust to changes in lighting. 

After filtering the input image, we will obtain a binary mask which will show in white all the pixels which survived (i.e., the red objects in the original image). In order to remove spurious pixels which have passed the filter, two consecutive erosions, followed by two dilations were performed over the mask. This results on a sharp binary mask with well defined blobs. 

Now, instead of focusing on a predefined row of the image for the localization of the line, we will take advantage of *OpenCV* functionalities. In this sense, we will find all the contours appearing on the binary mask using the `cv2.findContours()` method. This method, if possible, will return a segmentation of all the blobs contained in the image, otherwise, it will return an empty list. This will allow us to define two scenarios:

- If no contours are detected, we will slowly begin to turn around to look for a red line.

-  If red blobs are detected we will take the biggest one (we assume the red line will always be the biggest blob) and compute its centroid. For this purpose, the moments will be used. In this scenario, once the centroid is computed, the chosen controller will be activated.

The preferred and optimized situation will be the latter. 

The processing procedure is summarized below:

![Processing Pipeline](https://github.com/brodgon/brodgon.github.io/blob/master/docs/fig%20.png?raw=true)

Note that output visualization has changed since last post. This time, since we are detecting the contours using OpenCV, this library allow us to draw them, resulting on a delineation of the line, which, for visualization purposes, is better and cleaner.

Having this information, we can start with the controller’s implementation.

## Controllers: The robot’s brain

Although an scenario-based controller might be useful for simple robots aimed to solve really controlled tasks, this approach is not enough for our labor. To try to improve the results, we will define several controllers based on **classic control theory**.

Contrary to scenario-based controllers, classic controllers take into account the error made between the ideal state of the robot and its actual state. This modifies the output in order to minimize such error, making the robot to **oscillate** around the ideal state.

In this sense, among the most used controllers we highlight the proportional (P), integral (I) and derivative (D) controllers: 

-  **Proportional controllers**  react depending on the amount of error made, that is, they modulate the output in **proportion** to the error. The output is governed by a constant, known as the proportional gain ($K_{p}$) which modulate the error, such that: $ u = -K_{p}e + u_{b}$, where $u_{b}$ is the minimal action made (in our problem this term will be ignored).

- **Differential controllers** modulate the error considering the error made on the previous loop. This controller is based on the idea that a robot should not behave equally in case of descending or augmenting error. In this case, the output is modulated proportionally to de derivative of the error, that is, the difference between the error made before and the actual error. Just as before, the controller is governed by a constant, $K_{d}$, making the output to be defined as $u = K_{d} \frac{de}{dt}$.

- **Integral controllers** modulate the output considering the error made over all the iterations, deleting possible offsets: $u = K_{i} \int_{0}^{t} e(t)dt$.

The combination of these controllers leads to new controllers, like the **PD controllers** , defined by the addition of proportional and derivative controllers, or, the ones we are really interested on, the **PID controllers** , governed by the following mathematical expression:

$$ u = - K_{p}e - K_{d} \frac{de}{dt} - K_{i} \int_{0}^{t} e(t)dt $$

If gain constants are properly adjusted, PID controllers can result on a perfect performance robot without oscillations.  Constants will be adjusted by trial and error.

With this standard, we will employ a function for general PID controller and adjust the constants to modulate the angular velocity. The linear speed will depend on the output of the controller.

This idea, allowed the deployment of several trials.

### TRIAL I: P controller

Once the PID controller function is developed, the first controller tried was a simple P controller. 

For this purpose, we set the differential and integral gains to 0 and **adjust the proportional gain** (i.e., $K_{p}$) to modulate the angular velocity. The gain was adjust experimentally, realizing that, really small constants result in almost no effect on the robot, while big constants result on enormous oscillations.

Two cases were tried. Firstly, we set a constant linear speed (at 4) which, using a gain of 0.005 resulted in the completion of the circuit in 3:50 minutes. As seen on the gif below, The F1 shows some small fluctuations, but rapidly reach the origin. Although it is stable, in order to try to reduce the lap time, we set different linear speeds depending on the value of the angular velocity. In this case, circuit was completed in 3:40 min, that is, we gained 10, however, oscillations were bigger, probably, due to the linear speed shifts..

### TRIAL II: PD CONTROLLER

To try to improve the performance of simple P controllers, we decided to use the derivative term, that is, consider the evolution of the error. For this purpose, we left the integral term to 0 and adjust the proportional and derivative gains.

Just as before, we first try to maintain constant linear speed, and then, try to modulate it depending on the computed angular velocity. With a constant speed of 5.5, the circuit was completed in around 2:14 minutes with a pretty stable position. Its behaviour on curve was sharp, however, more fluctuations where seen on straight lines, where it took some more time to reach the equilibrium. On the other hand, changing linear speed show no real improvement, resulting in bigger fluctuations and, therefore, a bigger time (3:20min).


### TRIAL III: PID CONTROLLER

Last, but not least, we introduced the integral component, in order to try to obtain the perfect controller.  As always, constants were adjusted by trial and error for our two typical scenarios: constant and modulating linear speed. Surprisingly better results were obtained when using this type of controllers. Precisely, using the same parameters, we got laps of around 2 minutes, both for constant speed (1:48 min) and modulating linear speed (2:29 min). We realized integral component needed to be kept as small as possible to get better results.

As seen, behaviour is usually better when linear speed is mantained constant, showing less fluctuations in straight lines and smooth behaviours in curves. However, using this approach we are not getting all the potential of the car when we are positioned on a straight line. We will therefore need to find a way to solve this problem.


### TRIAL IV: CONTROL BASED ON POSITION

After a lot of trials, I realize that controlling linear speed depending on the angular velocity modulated by a single controller was a weak approach. When we increased linear speed, it drastically affected the next loops error, augmenting angular velocity and, therefore oscillations. 

To try to solve this, the best option I came with was to use two different controllers depending on the position of the F1, that is, depending on whether the car was on a straight or on a curved road. 

To reach this goal, the first step will therefore be to define what is consider straight and what is consider a curve. For this purpose, we will take advantage of the information given by the contour detected by OpenCV.  As discussed before, we have detected the centroid of the line’s blob using its moments; this computation results on a centroid which is somehow on the lowest part of the line (we have a robot which is continuously looking down). Using this point, we cannot determine if we are on a straight or curved line with enough time to react, therefore, other points need to be considered.

Since we have all the points delineating the line, we take the highest pixel detected, that is, the further point of the line detected by the robot. If its x coordinate is somehow aligned to the x coordinate of the centroid, it will mean we are on a straight line, and therefore, we can go a little bit faster, activating a **straight-line controller**. On the other hand, in case these points are not aligned, it will mean we are approaching a curve, so a **curve controller** will be activated, to react with enough time. 

To avoid possible circuit exercises, the straight line (or stable position) was constrained a little bit more: the car will only activate the fastest mode if we are around the ideal point.

The result of this approach is illustrated on the video below. As seen less fluctuations are made than in other scenarios, obtaining a time of between 1:50-2:03 min (depending on the simulation runed).

## Discussion: Flaws and improvements 
