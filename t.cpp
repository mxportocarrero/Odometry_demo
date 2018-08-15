#include <iostream>

// Incluir librerias standar
#include <stdio.h>
#include <stdlib.h>

// Incluimos el GLEW
#include <GL/glew.h>

// Incluimos el GLFW
#include <GLFW/glfw3.h>
GLFWwindow * window;

// Include GLM
#include <glm/glm.hpp>

#include "common/shader.hpp"


using namespace std;

int main(){
   // Checking GLFW initialization
   if( !glfwInit() ){
      cout << "Failed to initialize GLFW\n";
      getchar();
      return -1;
   }

   // Some usefull flags but most of them are unrequired
   glfwWindowHint(GLFW_SAMPLES, 4);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(600,600, "Test",NULL,NULL);
	if( window == NULL){
      cout << "Failed to Open GLFW window\n";
      getchar();
      glfwTerminate();
      return -1;
	}
	glfwMakeContextCurrent(window);

	// Inicializamos GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f); // Color de fondo en su forma normalizada

	do{
      // clear the screen
      glClear( GL_COLOR_BUFFER_BIT );

      glfwSwapBuffers(window);
      glfwPollEvents(); // Revisa si se ha emitido algun evento

	}
	// Check if the ESC key was pressed or the window was closed
	while(glfwGetKey(window,GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         glfwWindowShouldClose(window) == 0);

	glfwTerminate();

	return 0;


}
