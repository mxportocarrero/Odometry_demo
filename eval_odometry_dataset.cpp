/**
Rotating Cubes in 3D in Perspective View

g++ eval_odometry_dataset.cpp common/shader.cpp -o eval_odometry_dataset -lGL -lGLEW -lglfw -O3 -mavx2

Tambien podemos verificar el makefile que realizes
**/

// Precision de las imagenes, podemos alternar entre floats o dobles
//#define Double_Precision

#ifdef Double_Precision
typedef double myNum;
#else
typedef float myNum;
#endif // Double_Precision

// Incluir librerias standar
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> // std::copy
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>

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

// Import Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions> // Para hacer uso de esta libreria
// tuve que modificar algunos includes en el source code que utiliza por q referenciaba
// #include <Eigen/Core> pero deberia ser en mi caso #include <eigen3/Eigen/Core>

#include "common/shader.hpp"
#include "common/register.hpp"
#include "common/linear_algebra_functions.hpp"
#include "common/utilities.hpp"
#include "common/tum_rgbd_datasets.hpp"


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

void printMat44(const Mat44 &mat, const char* name){
	std:: cout << name << ":\n";
	for(int i = 0; i < 4; ++i){
		for(int j = 0; j < 4; ++j){
			std::cout << mat.m[i][j] <<  " ";
		}
		std::cout << "\n";
	}
}

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
static inline __m256 dual_lincomb_AVX8(__m256 A01, const Mat44 &B){
	__m256 result;

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
	__m256 A23 = _mm256_loadu_ps(&A.m[2][0]); // Filas 2 y 3

	__m256 out01rows = dual_lincomb_AVX8(A01,B);
	__m256 out23rows = dual_lincomb_AVX8(A23,B);

	// Guardamos los valores calculados al sector de memoria especificado de salida
	_mm256_storeu_ps(&out.m[0][0], out01rows);
	_mm256_storeu_ps(&out.m[2][0], out23rows);
}

//Realizamos un enum para tener un mejor acceso a las variables
struct Quaternion{
	float data[4];
	float x() const {return data[0];}
	float y() const {return data[1];}
	float z() const {return data[2];}
	float w() const {return data[3];}

	Quaternion(const float &qx,const float &qy,const float &qz,const float &qw){
		data[0] = qx;
		data[1] = qy;
		data[2] = qz;
		data[3] = qw;
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
	};

	float Btemp[] = {
		 q.w(),	 q.z(),	-q.y(),	-q.x(),
		-q.z(),	 q.w(),	 q.x(),	-q.y(),
		 q.y(),	-q.x(),	 q.w(),	-q.z(),
		 q.x(),	 q.y(),	 q.z(),	 q.w()
	};

	// Copiamos los datos a nuestras funciones A y B
	// Preferred method to copy raw arrays in C++. works with all types
	std::copy(Atemp,Atemp+16,A.m[0]);
	std::copy(Btemp,Btemp+16,B.m[0]);

	dual_matmul_AVX8(RotationMatrix,A,B);

	return RotationMatrix;
}

// Funcion sobrecargada que acepta tambien un vector de traslación
Mat44 Quaternion_2_Mat44(const float &tx, const float &ty, const float &tz, const float &qx,const float &qy,const float &qz,const float &qw){
	// Creamos el quaternion
	Quaternion q(qx,qy,qz,qw);

	// Creamos las dos matrices según lo propuesto en: (Alternativa 1)
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
	Mat44 A,B,ResultMatrix;

	float Atemp[] = {
		 q.w(),	 q.z(),	-q.y(),	 q.x(),
		-q.z(),	 q.w(),	 q.x(),	 q.y(),
		 q.y(),	-q.x(),	 q.w(),	 q.z(),
		-q.x(),	-q.y(),	-q.z(),	 q.w()
	};

	float Btemp[] = {
		 q.w(),	 q.z(),	-q.y(),	-q.x(),
		-q.z(),	 q.w(),	 q.x(),	-q.y(),
		 q.y(),	-q.x(),	 q.w(),	-q.z(),
		 q.x(),	 q.y(),	 q.z(),	 q.w()
	};

	// Copiamos los datos a nuestras funciones A y B
	// Preferred method to copy raw arrays in C++. works with all types
	std::copy(Atemp,Atemp+16,A.m[0]);
	std::copy(Btemp,Btemp+16,B.m[0]);

	// Este producto de matrices nos devuelve una matriz de la forma 
	// [R 0]
	// [0 1]
	dual_matmul_AVX8(ResultMatrix,A,B);

	// Agregamos los valores de traslacion a nuestras matrices
	ResultMatrix.m[0][3] = tx;
	ResultMatrix.m[1][3] = ty;
	ResultMatrix.m[2][3] = tz;

	return ResultMatrix;
}

Mat44 TwistCoord_2_Mat44(const float &v1, const float &v2, const float &v3, const float &w1,const float &w2,const float &w3){
	Eigen::VectorXd xi(6);
	xi << -v1,-v2,-v3,-w1,-w2,-w3; // usamos los valores negativos por que necesitamos la matriz inversa

	Eigen::Matrix4d rbm = twistcoord2rbm(xi);

	Mat44 ResultMatrix;

	// Copying data into mat44
	for(int rows = 0; rows < 4; ++rows){
		for(int cols = 0; cols < 4; ++cols){
			ResultMatrix.m[rows][cols] = rbm(rows,cols);
		}
	}

	return ResultMatrix;
}

// settings
const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

// Global variables
// ------------------
// camera // View from Z axis
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

enum MotionType{RBM_mat,TCoord,Quad};

void read_inputfile(std::map<double,Mat44> & data, const char* filename, const int & motion_type){
	// Loading/Reading Groundtruth Data
	std::ifstream data_file;
	data_file.open(filename);
	//data_file.open("data/groundtruth_fr1_xyz.txt");

	std::string line;
	std::vector<std::string> temp;
	if(data_file.is_open()){
		while(std::getline(data_file,line)){
			// Detect whether or not it is reading a comment
			if(line[0] != '#'){
				temp = split(line,' ');
				switch(motion_type){
					case TCoord:{
						data[std::stod(temp[0])] = TwistCoord_2_Mat44(std::stof(temp[1]),std::stof(temp[2]),std::stof(temp[3]),std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]));
						break;
					}
					case Quad:{
						// Capturing Rotation data
						// Recordemos que la funcion push_back crea una copia del argumento y la almacena en sus vectores
						data[std::stod(temp[0])] = Quaternion_2_Mat44(std::stof(temp[1]),std::stof(temp[2]),std::stof(temp[3]),std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]),std::stof(temp[7]));
						//Quaternion q(std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]),std::stof(temp[7]));
						break;
					}
					default:{
						break;
					}
				} // fin del switch case
			}
		} // Fin del bucle while
	} // Fin del condicional exterior
}

// Align Function




int main(){
	// Iterating over all datasets(ds)

	std::ofstream fout;
	fout.open("output/results_out_fast2.txt",std::fstream::app);
	fout.precision(4);
	fout << "# dataset_name rmse mean min max\n";

	for(int ds = 0; ds < NoDatasets; ++ds){

		// RBM(Rigid Body Motion) will store all the Mat44 matrices with their respecting timestampfor every frame
		std::map<double,Mat44> RBM_gt;
		std::map<double,Mat44> RBM_data;

		//read_inputfile(RBM_gt, "data/groundtruth_fr1_room.txt", Quad);
		//read_inputfile(RBM_data, "data/odometry.txt", TCoord);
		read_inputfile(RBM_gt, groundtruths[ds], Quad);
		read_inputfile(RBM_data, out_fast2[ds], TCoord);

		// En el caso de nuestra data. primero tenemos que acumularla para los puntos esten referenciados a mismo frame coordinate
		// Iteramos sobre todo nuestros datos y vamos acumulando las matrices para conocer su movimiento absoluto
		{ // Loooping throught the elements
			Mat44 AbsoluteRBM;
			float ARBM[] = {
				1,	0,	0,	0,
				0,	1,	0,	0,
				0,	0,	1,	0,
				0,	0,	0,	1
			};

			std::copy(ARBM,ARBM+16,AbsoluteRBM.m[0]);

			std::map<double,Mat44>::iterator it = RBM_data.begin();++it; int i;
			// Comenzamos desde 1 porque la primera matriz es correcta, es decir la identidad
			for(i = 1; i < RBM_data.size(); ++i, ++it){

				Mat44 tempResult;

				//dual_matmul_AVX8(tempResult,AbsoluteRBM,it->second);
				dual_matmul_AVX8(tempResult,AbsoluteRBM,it->second);

				// Copiamos el resultado de tempResult en la matriz Absoluta
				_mm256_storeu_ps(&AbsoluteRBM.m[0][0], _mm256_loadu_ps(&tempResult.m[0][0]));
				_mm256_storeu_ps(&AbsoluteRBM.m[2][0], _mm256_loadu_ps(&tempResult.m[2][0]));

				// Copy data into the map
				for(int rows = 0; rows < 4; ++rows){
					for(int cols = 0; cols < 4; ++cols){
						it->second.m[rows][cols] = AbsoluteRBM.m[rows][cols];
					}
				}
			}
		} // Fin de encontrar las transformaciones absolutas

		// Asociamos los datos, de acuerdo a sus timestamps
		// Vamos a iterar para encontrar los tiempos que sean correspondientes(que esten asociados)
		//std::cout << "Associating data:\n";
		std::vector<double> v_gt,v_data;
		{
			double max_distance = 0.02; // maxima separacion entre las asociaciones
			std::map<double,Mat44>::iterator it_gt = RBM_gt.begin();

			for(int i = 0; i < RBM_gt.size(); ++i, ++it_gt){
				std::map<double,Mat44>::iterator it_data = RBM_data.begin();
				for(int j = 0; j < RBM_data.size();++j, ++it_data){
					// Verificamos q la distancia en el tiempo no sea mucha
					if(abs(it_gt->first - it_data->first) < max_distance &&
						std::find(v_gt.begin(),v_gt.end(),it_gt->first) == v_gt.end() &&
						std::find(v_data.begin(),v_data.end(),it_data->first) == v_data.end()){
						v_gt.push_back(it_gt->first);
						v_data.push_back(it_data->first);
					}
				}
				//std::cout << i / (float)RBM_gt.size() << " %\n";
			}

		}

		// Verificamos los datos asociados // DESCOMENTAR
		// std::cout << "Asociated data\n";
		// for(int i  = 0; i < v_gt.size(); ++i){
		// 	std::cout.precision(15);
		// 	std::cout << i << " " << v_gt[i] << " " << v_data[i] << " ";
		// 	if(i < v_gt.size() - 1){
		// 		std::cout.precision(3);
		// 		std::cout << v_gt[i+1] - v_gt[i] << std::endl;
		// 	}
		// }

		std::vector<Eigen::Vector3d> tmp_coord_gt;
	    std::vector<Eigen::Vector3d> tmp_coord_data;

		GLfloat vertices[v_gt.size()*3];

		{ // Loooping throught the elements
			for(int i = 0; i < v_gt.size(); ++i){
				vertices[i*3+0] = RBM_gt[v_gt[i]].m[0][3];
				vertices[i*3+1] = RBM_gt[v_gt[i]].m[1][3];
				vertices[i*3+2] = RBM_gt[v_gt[i]].m[2][3];
				tmp_coord_gt.push_back(Eigen::Vector3d(vertices[i*3+0],vertices[i*3+1],vertices[i*3+2]));
			}
		}

		GLfloat vertices_data[v_data.size()*3];

		{ // Loooping throught the elements
			for(int i = 0; i < v_data.size(); ++i){
			//for(i = 0, it = RBM_data.begin(); i < 20; ++i, ++it){
				//printMat44(it->second,"mat**");
				vertices_data[i*3+0] = RBM_data[v_data[i]].m[0][3];
				vertices_data[i*3+1] = RBM_data[v_data[i]].m[1][3];
				vertices_data[i*3+2] = RBM_data[v_data[i]].m[2][3];

				tmp_coord_data.push_back(Eigen::Vector3d(vertices_data[i*3+0],vertices_data[i*3+1],vertices_data[i*3+2]));
			}

			// Calculamos nuestra matriz de transformacion, de alineamiento
			Eigen::Matrix4d T = QuickTransformation(tmp_coord_gt, tmp_coord_data);

			for(int i = 0; i < v_data.size(); ++i){
				Eigen::Vector4d tmp = T * Eigen::Vector4d(vertices_data[i*3+0],vertices_data[i*3+1],vertices_data[i*3+2],1.0);
				vertices_data[i*3+0] = tmp(0);
				vertices_data[i*3+1] = tmp(1);
				vertices_data[i*3+2] = tmp(2);
			}
		} // Fin de bucle

		//Calculando el traslational error RMSE
		Eigen::VectorXd distances(v_data.size());
		for(int i = 0; i < v_data.size(); ++i){
			Eigen::Vector3d tdata(vertices_data[i*3+0],vertices_data[i*3+1],vertices_data[i*3+2]);
			//std::cout << i << " groundtruth: "<<tmp_coord_gt[i](0) << "," << tmp_coord_gt[i](1) << "," << tmp_coord_gt[i](2) << " data: "
			//		  << vertices_data[i*3+0] << "," << vertices_data[i*3+1] << "," << vertices_data[i*3+2];
			Eigen::Vector3d tmp = tmp_coord_gt[i] - tdata;
			double d = sqrt( tmp.dot(tmp) );
			//std::cout << " dist. " << d << std::endl;
			distances(i) = d;
		}

		
		std::string ds_name(groundtruths[ds]);

		// EVALUATION PARAMETERS
		// ---------------------
		
		fout << ds + 1<< " " << ds_name.substr(29, ds_name.size()-29-4) << " "
							<< sqrt( (distances.dot(distances)) / (double)v_data.size()) << " "
							<< distances.mean() << " "
		//std::cout << "median " << distances.median() << " m\n";
		//std::cout << "std " << rmse << " m\n";
							<< distances.minCoeff() << " "
							<< distances.maxCoeff() << "\n";

		
	} // Fin del for de datasets

	fout.close();
}