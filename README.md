# Prerequisites
- [MinGW (32 bit)](https://sourceforge.net/projects/mingw/files/latest/download)
- Clion

## Why not use Cygwin?

There is a reason for the fact that we explicitly state that MinGW __must__ be used instead of Cygwin.
When you compile and run the source with Cygwin, you will get the following error: 

*freeglut (/some/path/): failed to open display ""*. 

This error is explained in 
[this](http://stackoverflow.com/questions/29595462/opengl-glut-cygwin-failed-to-open-display) post.
For completeness, we post the answer here:

"When compiling with Cygwin the programs you expect a "unix-ish" kind of environment. 
Graphics is done by an X server to which clients connect to on the one end, and a graphics driver attaches to the other end; 
the Cygwin X server uses the Windows GDI (and a very baseline OpenGL profile) for its backend. 
The error get is the program telling you, that it expects to be able to connect to an X server, but that it can't find one.
And when it comes to OpenGL on Windows, you should not use one! 

When building a program the uses OpenGL: __Don't use the Cygwin toolchain!__

Instead use the MinGW toolchain targeting the native Windows graphics APIs; you'll also need the appropriate builds for the relevant libraries. 
You can install the MinGW toolchain inside your Cygwin environment, but personally I prefer to have a self contained, 
independent installation."

Although we can fix this *failed to open display* problem by 
installing some additional Cygwin libraries and
[XLaunch/Xming](https://unix.stackexchange.com/questions/227889/cygwin-on-windows-cant-open-display)
, we were still not able to execute the binary as it gave rise to another error which we were not able to fix.

In short, just stick with MinGW :)

