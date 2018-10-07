// Asumimos que todas las librerias necesarias estan incluidas


// Uniform Variable Registration on Shaders

void RegisterUniform4fv(GLuint& programID, const char* var_name, const glm::vec4& vec){
	int location = glGetUniformLocation(programID,var_name);
	glUniform4fv(location,1,glm::value_ptr(vec));
}

void RegisterMatrix4fv(GLuint& programID, const char* var_name, const glm::mat4& mat){
	int loc = glGetUniformLocation(programID, var_name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(mat));
}