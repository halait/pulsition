# Pulsition
A real-time rigid body 2D physics engine for the web. Development is still active and many features are being added including arbitrary convex polygons. Checkout my game at [https://buildhead.web.app/](https://buildhead.web.app/) to see it in action.

## Overview
Pulsition simulates physics using a local solver to solve pairwise contraints. Many iterations are needed to solve competing constraints. Pulsition does not support variable-rate time steps. Only two shapes are currently supported Circle and Obround. Convex polygons are available but not completely supported. Axis-aligned bounding boxes are supported to observe the position of objects. Warm-starting is used by default to significantly reduce the number of iterations needed to solve competing constraints at the cost of accuracy. Speculative contacts are used by default to prevent tunneling also at the cost of accuracy. There are two main type of constraints supported collisions and joints. Colliding pairs can be filtered using groups. Joints can be motorized. Pulsition does not render graphics and there is no testing UI. I recommend using WebGL for graphics rendering. If you require basic rendering for testing, please contact me.

## Install
Download and unzip the master branch. In the unzipped folder you will find `physics.js`, include it in your website using the `script` tag.

## Glossary
Body: a rigid body that Pulsition attempts to simulate.
<!--
## Basic tutorial
Pulsition is implemented as one very large object called `pw`. Many variables are set to sensible defaults such as `G` the number that represents the acceleration of gravity. To set `G` to 0.1 we can simply write:
```javascript
pw.G = 0.1;
``` 
To create a body we call [`pw.create()`](#pw.create()) with one parameter, a [body definition](#body-definition). The body definition specifies the properties of the body being created.  Let's create a circle body by writing:
 ```javascript
 let circleRef = pw.create({
  form: pw.CIRCLE_FORM,
  type: pw.MOVABLE_TYPE,
  x: 0,
  y: 0,
  radius: 0.1,
  density: 1,
  group: 0,
  userFloats: [42, 3.14],
  staticFriction: 0.9,
  kineticFriction: 0.8,
  linearVelocityResistance: 0.98,
  rotationalVelocityResistance: 0.98,
});
 ```
 The `pw.create()` method returns a number that is used to refer to the body within `pw`. To simulate we call `pw.update()` repeatedly at fixed time steps usually just before drawing to the screen. To do: the rest of the tutorial.
 -->

## Reference
### `pw.create()`
The `pw.create()` method creates bodies that are simulated by Pulsition.
#### Example
```
pw.create({
  form: pw.CIRCLE_FORM,
  type: pw.MOVABLE_TYPE,
  x: 0,
  y: 0,
  radius: 0.1,
  density: 1,
  group: COPLANAR_GROUP,
  userFloats: [0, 1, 2, 3],
  staticFriction: 0.9,
  kineticFriction: 0.8,
  linearVelocityResistance: 0.98,
  rotationalVelocityResistance: 0.98,
});
```
#### Syntax
```
pw.create(def)
```
##### Parameters
`def`: the [body definition](#body-definition).
##### Return value
The body reference, used in all subsequent calls to any methods within `pw` to refer to the body created.

### Body Definition
The body definition is the object that is passed to the `pw.create()` method to define the properties of the body being created. Any unrecognized properties are ignored. It must contain the following properties:\
`form`: either `pw.CIRCLE_FORM`, `pw.PLANE_FORM`, `pw.POLYGON_FORM` or `pw.AABB_FORM`. `pw.CIRCLE_FORM` creates a circular body. `pw.PLANE_FORM` creates an obround body. `pw.POLYGON_FORM` is not fully supported. `pw.AABB_FORM` creates an axis-aligned bounding box that is not physically simulated. If `pw.CIRCLE_FORM` then `x`, `y` and `radius` must also be specified. If `pw.PLANE_FORM` then `width` and `vertices` must also be specified. If `pw.AABB_FORM` then `vertices` must also be specified. If `pw.POLYGON_FORM` then `vertices` must also be specified.\
`type`: either `pw.MOVABLE_TYPE` or `pw.FIXED_TYPE`. `pw.MOVABLE_TYPE` specifies a normal body that can move. `pw.FIXED_TYPE` specifies a object that will never move, suitable for simulating massive objects like the earth. If `pw.MOVABLE_TYPE` then `density` must also be specified.\
It may contain the following properties:\
`density`: a number that specifies the density of the body, it must be postive.\
`group`: a number that specifies the group of the body. Used to filter colliding pairs.\
`userFloats`: a array of numbers. Can contain any numbers and be any length. The array can be retreived with `pw.getUserFloats()` and modified with `pw.setUserFloats()` though length cannot be changed after the body is created\
`staticFriction`: a number that is used to calculate the amount of static friction between two colliding pairs, it must be positive. The static friction between two bodies is calculated by the average of their `staticFriction`.
`kineticFriction`: a number that is used to calculate the amount of kinetic friction between two colliding pairs, it must be positive. The kinetic friction between two bodies is calculated by the average of their `kineticFriction`.
`linearVelocityResistance`: a number that specifies the amount of dampening applied to linear velocity of the body per time step. It must be greater than or equal to `0` and less than or equal to `1`. It is used to simulate things such as air resistance. The closer it is to `0` the more resistance will be applied. If `0` then the object will not move linearly.
`rotationalVelocityResistance`: a number that specifies the amount of dampening applied to rotational velocity of the body per time step. It must be greater than or equal to `0` and less than or equal to `1`. It is used to simulate things such as rolling resistance. The closer it is to `0` the more resistance will be applied. If `0` then the object will not rotate.


`x`: a number that specifies the position of the center of the body in the x-axis.\
`y`: a number that specifies the position of the center of the body in the y-axis.\
`radius`: a number that specifies the radius of the body, it must be positive.\
`width`: a number that specifies the width of the body, it must be a positive.\
`vertices`: a 2D array that specifies the positions of the vertices of the body. The `length` of each subarray must be equal to `2`, each element a number that specifies a position in the x and y axis respectively. If the `form` property of the body definition is equal to `pw.PLANE_FORM` then the length of `vertices` must be equal to `2`, each subarray specifying the position of the two ends of the body. If the `form` property of the body definition is equal to `pw.POLYGON_FORM` then the length of `vertices` must be more than `3`, each subarray specifying the position of each vertex of the body in counter-clockwise direction. If the `form` property of the body definition is equal to `pw.AABB_FORM` then the length of `vertices` must be equal to `2`, each subarray specifying the position of the minimum and maximum vertex respectively.\


If you want to discuss or use this repository please contact me.
