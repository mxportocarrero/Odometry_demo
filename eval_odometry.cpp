/**
Rotating Cubes in 3D in Perspective View

g++ eval_odometry.cpp common/shader.cpp -o eval_odometry -lGL -lGLEW -lglfw

Tambien podemos verificar el makefile que realizes
**/

// Incluir librerias standar
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> // std::copy
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Include SSE/AVX2 Libraries
// -------------------
#include <immintrin.h>

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
#include "common/utilities.hpp"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// Declaring some Matrix Arithmetic.
// examples taken from https://gist.github.com/rygorous/4172889

// La razón por la que declaramos nuestra matrix como union, es para poder interpretar nuestra
// data tanto como un doble arreglo de floats o un arreglo de vectores intrinsecos para SIMD
union Mat44 {
    float m[4][4];
    __m128 row[4];
};

// Tengamos en cuenta que una multiplicación de matrices puede llevarse a cabo de diferentes formas
// en nuestro caso, expresamos este proceso mediante combinaciones lineales(linear combination). Ver agenda
static inline __m128  lincomb_SSE(const __m128 &a, const Mat44 &B){
	__m128 result;
	// Usamos el shuffle para realizar un broadcast de una variable
	// (copiar un valor a todos los espacios de un mm128). mas info en http://www.songho.ca/misc/sse/sse.html
	// ------------------------------------------------------------------------------------------------------
	result = _mm_mul_ps(_mm_shuffle_ps(a,a, 0x00), B.row[0]);
	result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a,a, 0x55), B.row[1]));
	result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a,a, 0xaa), B.row[2]));
	result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a,a, 0xff), B.row[3]));
	return result;	
}

void matmul_SSE(Mat44 &out, const Mat44 &A, const Mat44 &B){
	__m128 out0row =  lincomb_SSE(A.row[0],B);
	__m128 out1row =  lincomb_SSE(A.row[1],B);
	__m128 out2row =  lincomb_SSE(A.row[2],B);
	__m128 out3row =  lincomb_SSE(A.row[3],B);

	out.row[0] = out0row;
	out.row[1] = out1row;
	out.row[2] = out2row;
	out.row[3] = out3row;
}

// Esta funcion usa los intrinsecos AVX, aprovechando la extension a 256 bits de los registros YMM
// Como son matrices 4x4, podemos aprovechar ello y hacer calculos duales
// Es decir de 2 en 2 filas
static inline __mm256 dual_lincomb_AVX8(__m256 A01, const Mat44 &B){
	__mm256 result;

	result = _mm256_mul_ps(_mm256_shuffle_ps(A01,A01, 0x00), _mm256_broadcast_ps(&B.row[0]));
	result = _mm256_add_ps(result, _mm256_mul_ps(_mm256_shuffle_ps(A01,A01, 0x55), _mm256_broadcast_ps(&B.row[1])));
	result = _mm256_add_ps(result, _mm256_mul_ps(_mm256_shuffle_ps(A01,A01, 0xaa), _mm256_broadcast_ps(&B.row[2])));
	result = _mm256_add_ps(result, _mm256_mul_ps(_mm256_shuffle_ps(A01,A01, 0xff), _mm256_broadcast_ps(&B.row[3])));
	return result;
}

void dual_matmul_AVX8(Mat44 &out, const Mat44 &A, const Mat44 &B){
	_mm256_zeroupper(); // Limpiamos los registros ymm

	// Cargamos los valores al registro, usamos la funcion para datos no alineados
	__m256 A01 = _mm256_loadu_ps(&A.m[0][0]); // Filas 0 y 1
	__m256 A23 = _mm256_loadu_ps(&A.m[0][0]); // Filas 2 y 3

	__m256 out01rows = dual_lincomb_AVX8(A01,B);
	__m256 out23rows = dual_lincomb_AVX8(A23,B);

	// Guardamos los valores calculados al sector de memoria especificado de salida
	_mm256_storeu_ps(&out.m[0][0], out01rows);
	_mm256_storeu_ps(&out.m[0][0], out23rows);
}

//Realizamos un enum para tener un mejor acceso a las variables
struct Quaterion{
	float data[4];
	float w(){return data[0];}
	float x(){return data[1];}
	float y(){return data[2];}
	float z(){return data[3];}

	quaternion(const float &w,const float &x,const float &y,const float &z){
		data[0] = w;
		data[1] = x;
		data[2] = y;
		data[3] = z;
	}
};
// Conversion functions
// Funcion que devuelve el objeto Mat44 a partir de un arreglo de 4 floats(unit quaternion)
Mat44 Quaternion_2_Mat44(const Quaternion &q){
	// Creamos las dos matrices según lo propuesto en: (Alternativa 1)
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
	Mat44 A,B,RotationMatrix;

	float Atemp[] = {
		 q.w(),	 q.z(),	-q.y(),	 q.x(),
		-q.z(),	 q.w(),	 q.x(),	 q.y(),
		 q.y(),	-q.x(),	 q.w(),	 q.z(),
		-q.x(),	-q.y(),	-q.z(),	 q.w()
	}

	float Btemp[] = {
		 q.w(),	 q.z(),	-q.y(),	-q.x(),
		-q.z(),	 q.w(),	 q.x(),	-q.y(),
		 q.y(),	-q.x(),	 q.w(),	-q.z(),
		 q.x(),	 q.y(),	 q.z(),	 q.w()
	}

	// Copiamos los datos a nuestras funciones A y B
	// Preferred method to copy raw arrays in C++. works with all types
	std::copy(Atemp,Atemp+16,A.m[0]);
	std::copy(Btemp,Btemp+16,B.m[0]);

	dual_matmul_AVX8(RotationMatrix,A,B);

	return RotationMatrix;
}

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
	std::vector<quaternion> v;

	// Loading Groundtruth Data
	std::ifstream data_file;
	data_file.open("data/groundtruth.txt");

	std::string line;
	std::vector<std::string> temp;
	if(data_file.is_open()){
		while(std::getline(data_file,line)){
			// Detect whether or not it is reading a comment
			if(line[0] != '#'){
				temp = split(line,' ');
				v.push_back(Quaternion(std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]),std::stof(temp[7])));
			}
		} // Fin del bucle while
	} // Fin del condicional exterior
	



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

	// Etapa iterativa de la ventana

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