Dechode's Godot-FFB-SDL GDExtension as ready to use-version (for Windows).
Original version on GitHub: https://github.com/Dechode/Godot-FFB-SDL
This version was compiled from this: https://github.com/Dechode/Godot-FFB-SDL/tree/23849cbe43750153cec22f9b6494293813cc45f2

In windows this should work just by copying the directory "Godot-FFB-SDL-Extension" into your project directory.
In Linux I couldn't get this working. Compiled nicely (.so-file can be found in "bin"-directory), but trying to use led Godot to report this error: "Can't open dynamic library: /home/user/Documents/Godot-FFB-Test/Godot-FFB-SDL-Extension/bin/libffbplugin.linux.template_debug.x86_64.so. Error: /home/user/Documents/Godot-FFB-Test/Godot-FFB-SDL-Extension/bin/libffbplugin.linux.template_debug.x86_64.so: undefined symbol: SDL_HapticClose." and subsequent other errors. This was in Linux Mint 21.3 Virginia. AFAIK I managed to install SDL2 successfully so not sure what went wrong. As I'm not very experienced with Linux this might be just some simple user error, though.

Tested with Logitech Driving Force GT and windows 10. Worked flawlessly when run from the editor. Exported project not tested.
