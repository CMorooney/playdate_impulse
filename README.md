wip playdate impulse engine maybe? C SDK

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
