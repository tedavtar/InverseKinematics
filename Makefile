CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iexample_00/glut-3.7.6-bin
	LDFLAGS = -lglut -lGL
endif
	
RM = /bin/rm -f 
all: main 
main: example_00/example_00.o 
	$(CC) $(CFLAGS) -o as0 example_00/example_00.o $(LDFLAGS) 
example_00/example_00.o: example_00/example_00.cpp
	$(CC) $(CFLAGS) -c example_00/example_00.cpp -o example_00/example_00.o
clean: 
	$(RM) *.o example_00/*.o as0
 

