# suBox2D

suBox2D is a simple wrapper around Box2D physics for use with the Cinder framework.

Wraps Box2D version 2.3.1  
Builds against [Cinder glNext](https://github.com/cinder/cinder/tree/glNext)  
Clone into your cinder/blocks/ directory to get started.

## Features
- Sandbox for easier world and body creation
- Smart pointers for managing the lifetime of physics bodies
- Scale for converting between world and screen space
- SimpleControl for easily setting up basic mouse interaction
- ContactListener to simplify routing of physics event handling
- Debug Renderer following Box2D convention
- Project template for using Box2D in Cinder projects.

## Purpose
- Ease Box2d world setup and entity creation
- Simplify proper coordinate conversion
- Simplify Box2d entity lifetime management
- Minimize additional interfaces to learn beyond Box2d and C++11

Box2D is copyright Erin Catto. The source and license of Box2D are in src/Box2D
