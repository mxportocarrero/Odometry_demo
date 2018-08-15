all:
	g++ t.cpp common/shader.cpp -o test -lGLEW -lglfw -lGL -lGLU
