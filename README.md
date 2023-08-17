wip playdate impulse engine maybe? C SDK

this project is/was meant to be a clean look at adding extended collision behavior to the Playdate SDK in such a way that making a pinball game might be supported.

------------

running the project will get you this:
![](https://github.com/CMorooney/playdate_impulse/blob/master/screenshots/sample.gif)

------------

todo:
- [ ] actual/bettery AABB-triangle collision check
- [ ] Circle rotation
- [ ] OOB/box rotation
- [ ] triangle rotation

at least kinda working:
- [x] circle-circle collision
- [x] circle-AABB collision
- [x] circle-triangle (edge) collision
- [x] AABB-AABB collision

-------------

to get started:

from root (if build directory doesn't exist:
```mkdir build && cd build && cmake ..```

to build:
from build directory:
```make```

this will create pinball.pdx at repo root which can be opened in simulator.
so, from build directory in macos I usually just `make && open ../impulse.pdx` for iterating

---------

clangd LSP:
`compile_commands.json` gets created in the `build` directory but clangd lsp expects it to be at repo root, so don't forget to create a symlink
