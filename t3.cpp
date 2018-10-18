/**
Rotating Cube

g++ t3.cpp common/shader.cpp -o test3 -lGL -lGLEW -lglfw

Tambien podemos verificar el makefile que realizes
**/


#include <iostream>

// Incluir librerias standar
#include <stdio.h>
#include <stdlib.h>

// Incluimos el GLEW
// Esta libreria nos sirve para aumentar algunas funciones que no vienen por defecto
// con las librerias. Otra similar es glad
#include <GL/glew.h>

// Incluimos el GLFW
#include <GLFW/glfw3.h>
GLFWwindow * window;

// Include GLM
#define GLM_FORCE_RADIANS // poner este header para que la funcion rotate no se queje
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "common/shader.hpp"
#include "common/register.hpp"


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

	//
	// Registering the VAOs
	//
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID); // El primer argumento debe ser el numero de VAOs
	glBindVertexArray(VertexArrayID);

	// Creamos y compilamos nuestro programa GLSL a partir de los shaders
	GLuint programID = LoadShaders("shaders/t3.vertexshader", "shaders/t3.fragmentshader");

	static const GLfloat vertices[] = {
		// first triangle
		0.5f, 0.5f, 0.0f,
		0.5f, -0.5f, 0.0f,
		-0.5f,  0.5f, 0.0f,
		// second triangle
		0.5f, -0.5f, 0.0f,
		-0.5f, -0.5f, 0.0f,
		-0.5f,  0.5f, 0.0f,
	};

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	//
	// Fin de la Etapa de Registro
	//

	do{
      // clear the screen
      glClear( GL_COLOR_BUFFER_BIT );

      // Use our shader
      glUseProgram(programID);

      // updating a uniform color
      float timeValue = glfwGetTime();
      float greenValue = sin(timeValue) / 2.0f + 0.5f;
      glm::vec4 vec(0.0f, greenValue, 0.0f, 1.0f);
      RegisterUniform4fv(programID,"ourColor",vec);
      //int vertexColorLocation = glGetUniformLocation(programID,"ourColor");
      //glUniform4f(vertexColorLocation, 0.0f, greenValue, 0.0f, 1.0f);

      // rotating the rectangle
      glm::mat4 trans(1.0f); // Debemos inicializar asi para que no deje de renderizar
      //trans = glm::rotate(trans, glm::radians(45.0f), glm::vec3(0.0f,0.0f,1.0f));
      // actualizando los valores podemos rotar el cuadrado en el tiempo
      trans = glm::rotate(trans, (float)glfwGetTime(), glm::vec3(0.0f,0.0f,1.0f));
      // Algunas notas sobre la rotacion
      // el angulo o cantidad de rotacion debe encontrarse en radianes
      // el ultimo parametro es el vector de rotacion, DEBE SER UNITARIO, normalizar antes de ponerlo
      // Las matrices se acumulan automaticamente
      trans = glm::scale(trans, glm::vec3(0.5f,0.5f,0.5f));
      // Mandamos los valores a los shaders
      int transformLoc = glGetUniformLocation(programID,"transform");
      glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(trans));

      /**Drawing stage**/
      // 1st attribute buffer : vertices
      glEnableVertexAttribArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
      glVertexAttribPointer(
         0,          // attribute 0. No particular reason for 0, but must match the layout in the shader
         3,          // size
         GL_FLOAT,   // type
         GL_FALSE,   // normalized?
         0,          // stride
         (void*)0    // array buffer offset
      );

      //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Para dibujar solo en wireframe

      glDrawArrays(GL_TRIANGLES, 0, 6); // 3 indices starting at 0 -> 1 triangle

      glDisableVertexAttribArray(0);

      /**Final Stage: Refreshing buffers**/
      glfwSwapBuffers(window);
      glfwPollEvents(); // Revisa si se ha emitido algun evento

	}
	// Check if the ESC key was pressed or the window was closed
	while(glfwGetKey(window,GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         glfwWindowShouldClose(window) == 0);

   // Cleanup VBO
   glDeleteBuffers(1, &vertexbuffer);
   glDeleteVertexArrays(1, &VertexArrayID);
   glDeleteProgram(programID);

	glfwTerminate();

	return 0;


}
