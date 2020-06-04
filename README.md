# pulsition
A fast and open-source 2d physics engine for the web.

To see it in action checkout my game at [https://js-physics-game.web.app/](https://js-physics-game.web.app/).

This physics engine is made specifically for use in the browser and thus written in JS (perhaps ported to WASM in the future).
Due to most JS engines being slow accessing variables relative to native running code I am experimenting using a large typedArray 
(Float64Array) to store data rather than objects to improve performance. There is ~20% speed boost in V8 so I am sticking with it 
for now. I emulated C/C++ style of manual memory management with the typedArray serving as memory.

If you want to discuss or use this repo I would be happy to talk to you.
