/**
Rotating Cubes in 3D in Perspective View

g++ t4.cpp common/shader.cpp -o test4 -lGL -lGLEW -lglfw

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

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

// Global variables
// ------------------
// camera
glm::vec3 cameraPos   = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
float yaw   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;
float lastX =  800.0f / 2.0;
float lastY =  600.0 / 2.0;
float fov   =  45.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


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
	window = glfwCreateWindow(600,600, "3D Free Navigation",NULL,NULL);
	if( window == NULL){
      cout << "Failed to Open GLFW window\n";
      getchar();
      glfwTerminate();
      return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


	// Inicializamos GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// configure global opengl state
	// ------------------------------
	glEnable(GL_DEPTH_TEST);

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);


	// Creamos y compilamos nuestro programa GLSL a partir de los shaders
	GLuint programID = LoadShaders("shaders/t4.vertexshader", "shaders/t4.fragmentshader");

	static const GLfloat vertices[] = {
		-0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,

        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,

         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,

        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f,
	};

	// world space positions of our cubes
    glm::vec3 cubePositions[] = {
        glm::vec3( 0.0f,  0.0f,  0.0f),
        glm::vec3( 2.0f,  5.0f, -15.0f),
        glm::vec3(-1.5f, -2.2f, -2.5f),
        glm::vec3(-3.8f, -2.0f, -12.3f),
        glm::vec3( 2.4f, -0.4f, -3.5f),
        glm::vec3(-1.7f,  3.0f, -7.5f),
        glm::vec3( 1.3f, -2.0f, -2.5f),
        glm::vec3( 1.5f,  2.0f, -2.5f),
        glm::vec3( 1.5f,  0.2f, -1.5f),
        glm::vec3(-1.3f,  1.0f, -1.5f)
    };

	//
	// Registering the VAOs
	//
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID); // El primer argumento debe ser el numero de VAOs
	glBindVertexArray(VertexArrayID);

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	//
	// Fin de la Etapa de Registro
	//

	do{
		// per-frame time logic
		float timeValue = glfwGetTime();
		deltaTime = timeValue - lastFrame;
		lastFrame = timeValue;

		// Consultamos los inputs
		processInput(window);


		// RENDER:
		// --------
		// Dark blue background
		glClearColor(0.0f, 0.0f, 0.4f, 0.0f); // Color de fondo en su forma normalizada
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);

		// Programming our View and Projection matrices
		glm::mat4 view(1.0f);
		view       = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		RegisterMatrix4fv(programID,"view",view);

		glm::mat4 projection(1.0f);
		projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		RegisterMatrix4fv(programID,"projection",projection);



		// updating a uniform color
		float greenValue = sin(timeValue) / 2.0f + 0.5f;
		glm::vec4 vec(0.0f, greenValue, 0.0f, 1.0f);
		RegisterUniform4fv(programID,"ourColor",vec);
		//int vertexColorLocation = glGetUniformLocation(programID,"ourColor");
		//glUniform4f(vertexColorLocation, 0.0f, greenValue, 0.0f, 1.0f);

		// rotating the rectangle
		glm::mat4 trans;	
		// actualizando los valores podemos rotar el cuadrado en el tiempo
		trans = glm::rotate(trans, (float)glfwGetTime(), glm::vec3(0.0f,0.0f,1.0f));
		trans = glm::scale(trans, glm::vec3(0.5f,0.5f,0.5f));
		// Mandamos los valores a los shaders
		RegisterMatrix4fv(programID,"transform",trans);

		/**Drawing stage**/
		// 1st attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
        	0,          // attribute 0. No particular reason for 0, but must match the layout in the shader
        	3,          // size
        	GL_FLOAT,   // type
        	GL_FALSE,   // normalized?
        	3 * sizeof(float),          // stride
        	(void*)0    // array buffer offset
        );

        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Para dibujar solo en wireframe

        for(int i = 0 ; i < 10; ++i){
        	// Calculamos la matriz modelo para cada objeto segun nuestro array
        	glm::mat4 model(1.0f);
        	model = glm ::translate(model,cubePositions[i]);
        	float angle = 20.0f * i;
        	//model = glm::rotate(model, (float)glfwGetTime() *  glm::radians(50.f), glm::vec3(1.0f, 0.3f, 0.5f));
        	model = glm::rotate(model, (float)glfwGetTime() *  glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
        	RegisterMatrix4fv(programID,"model",model);

        	glDrawArrays(GL_TRIANGLES, 0, 36); // 3 indices starting at 0 -> 1 triangle
        }

        glDisableVertexAttribArray(0);

        /**Final Stage: Refreshing buffers**/
        glfwSwapBuffers(window);
        glfwPollEvents(); // Revisa si se ha emitido algun evento
	}
	// Check if the ESC key was pressed or the window was closed
	while(glfwGetKey(window,GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);

    // Cleanup VBO
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(programID);

	glfwTerminate();

	return 0;

}

// Callbacks

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.1f; // change this value to your liking
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    if (fov >= 1.0f && fov <= 45.0f)
        fov -= yoffset;
    if (fov <= 1.0f)
        fov = 1.0f;
    if (fov >= 45.0f)
        fov = 45.0f;
}