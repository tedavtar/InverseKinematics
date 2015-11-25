CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX -Iexample_00/eigen-eigen-10219c95fe65
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iexample_00/glut-3.7.6-bin -Iexample_00/eigen-eigen-10219c95fe65

	LDFLAGS = -lglut -lGL
endif
	
RM = /bin/rm -f 
all: main 
main: example_00/wtf.o 
	$(CC) $(CFLAGS) -o test example_00/wtf.o $(LDFLAGS) 
example_00/wtf.o: example_00/wtf.cpp
	$(CC) $(CFLAGS) -c example_00/wtf.cpp -o example_00/wtf.o
clean: 
	$(RM) *.o example_00/*.o test
 

