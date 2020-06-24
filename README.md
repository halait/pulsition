# Pulsition
A real-time rigid body 2D physics engine for the web. Development is still active and many features are being added including arbitrary convex polygons. Checkout my game at [https://buildhead.web.app/](https://buildhead.web.app/) to see it in action.

## Overview
Pulsition simulates physics using a local solver to solve pairwise contraints. Many iterations are needed to solve competing constraints. Pulsition does not support variable-rate time steps. Only two shapes are currently supported Circle and Obround. Convex polygons are available but not completely supported. Warm-starting is used by default to significantly reduce the number of iterations needed to solve competing constraints at the cost of accuracy. Speculative contacts are used by default to prevent tunneling also at the cost of accuracy. There are two main type of constraints supported collisions and joints. Colliding pairs can be filtered using groups. Joints can be motorized. Pulsition does not render graphics and there is no testing UI. I recommend using WebGL for graphics rendering. If you require basic rendering for testing, please contact me.

## Install
Download and unzip the master branch. In the unzipped folder you will find `physics.js` move it to the desired location and include it in your website using the `script` tag.  

## Basic tutorial
First, some terminology: physics object refers to some specific shaped object like circle or obround that Pulsition attempts to simulate. It is not related to an object in object-oriented programming.

Pulsition is implemented as one very large object called `pw`. To create a physics object we call `pw.create()` with one parameter, a definition. The definition specifies properties of the object being created.

## Reference
### `pw.create()`
The `pw.create()` method creates physics objects that are simulated by Pulsition.
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
`def`: the [physics object definition](#physics-object-definition).
##### Return value
A uniquely identifying number that can be used to refer to the physics object in subsequent method calls to `pw`, similiar to a pointer in C language.

### Physics Object Definition
The physics object definition is the object that is passed to the `pw.create()` method to create a physics object. It specifies the properties of the physics object being created. It must contain the following properties:\
`form`: either [`pw.CIRCLE_FORM`](#pw.circle_form), `pw.PLANE_FORM`, `pw.POLYGON_FORM` or `pw.AABB_FORM`. `pw.CIRCLE_FORM` is specified to create a circular object. If specified then `x`, `y` and `radius` must also be specified. `x` and `y` are numbers that specify the position and `radius` specifies the radius it must be a positive number of the physics object. `pw.PLANE_FORM` is specified to create a obround object. If specified then `width` and `vertices` must also be specified. `width` specifies the width of the physics object and `vertices` m     `pw.POLYGON_FORM` is not fully supported.
If `pw.POLYGON_FORM` or `pw.PLANE_FORM` is specified This will specifiy the shape of the physics object.\
`type`: either `pw.MOVABLE_TYPE` or `pw.FIXED_TYPE`. `pw.MOVABLE_TYPE` specifies a normal physics object that can move. If `pw.MOVABLE_TYPE` is specified then `density` must also be specified. `density` must be a positive number and represents the density of the physics object. `pw.FIXED_TYPE` specifies a object that will never move, suitable for simulating massive objects like the earth.\




























### `pw.CIRCLE_FORM`








If you want to discuss or use this repository please contact me.
