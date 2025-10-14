#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include"MeshLoader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

//读文件
static std::string readText(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error(std::string("Cannot open file: ") + p);
    std::ostringstream ss; 
    ss << f.rdbuf(); 
    return ss.str(); //fstream->sstream
}
// 编译/链接着色器
static GLuint buildProgram(const std::string& vsPath, const std::string& fsPath) {
    auto vsSrc = readText(vsPath);
    auto fsSrc = readText(fsPath);
    //着色器创建和编译
    auto compile = [](GLenum type, const std::string& src) {
        GLuint sh = glCreateShader(type);
        const char* p = src.c_str();
        glShaderSource(sh, 1, &p, nullptr);
        glCompileShader(sh);
        //检查编译器的编译状态
        GLint success; 
        glGetShaderiv(sh, GL_COMPILE_STATUS, &success);
        if (!success) {
            GLint logLen = 0;
            glGetShaderiv(sh, GL_INFO_LOG_LENGTH, &logLen);
            std::string infologshader;
            infologshader.resize(logLen > 0 ? logLen : 1);
            glGetShaderInfoLog(sh, static_cast<GLsizei>(infologshader.size()), nullptr, &infologshader[0]);
            throw std::runtime_error(std::string(type == GL_VERTEX_SHADER ? "VS" : "FS") + " compile failed:\n" + infologshader);
        }
        return sh;
    };
    GLuint vs = compile(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fsSrc);
    //创建项目以链接着色器
    GLuint program = glCreateProgram();
    glAttachShader(program, vs); 
    glAttachShader(program, fs);
    glLinkProgram(program);
    GLint ok;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint logLen = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLen);
        std::string infologprogram;
        infologprogram.resize(logLen > 0 ? logLen : 1);
        glGetProgramInfoLog(program, static_cast<GLsizei>(infologprogram.size()), nullptr, &infologprogram[0]);
        glDeleteShader(vs);
        glDeleteShader(fs);
        glDeleteProgram(program);
        throw std::runtime_error(std::string("Program link failed:\n") + infologprogram);
    }
    glDeleteShader(vs); 
    glDeleteShader(fs);
    return program;
}

int main()
{
    try {
        std::string objDir = "D:/experience/try/DasModel/3DModel/OBJ/Data/Tile_+001_+000";
        if (objDir.empty()) {
            std::cout << "No useful file find\n";
            return 0;
        }

        const std::string vsPath = "D:\\experience\\Cloud2Pano\\Cloud2Pano\\Cloud2Pano\\shader\\shader.vert";
        const std::string fsPath = "D:\\experience\\Cloud2Pano\\Cloud2Pano\\Cloud2Pano\\shader\\shader.frag";

        // quick existence checks with clearer messages
        if (!std::filesystem::exists(vsPath)) throw std::runtime_error(std::string("Vertex shader not found: ") + vsPath);
        if (!std::filesystem::exists(fsPath)) throw std::runtime_error(std::string("Fragment shader not found: ") + fsPath);

        GLFWwindow* window;

        if (!glfwInit())
            return -1;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
        if (!window)
        {
            glfwTerminate();
            return -1;
        }
        glfwMakeContextCurrent(window);

        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK) {
            std::cerr << "Failed to initialize GLEW" << std::endl;
            return -1;
        }

        GLuint program = buildProgram(vsPath, fsPath);
        glUseProgram(program);

        /*std::cout << "Checking objDir: " << objDir << std::endl;
        std::cout << "Exists: " << std::filesystem::exists(objDir) << ", IsDir: " << std::filesystem::is_directory(objDir) << std::endl;

        if (!std::filesystem::exists(objDir) || !std::filesystem::is_directory(objDir)) {
            std::cerr << "OBJ folder not found or not a directory: " << objDir << std::endl;
            return -1;
        }*/
        LoadedScene scene;
        try {
            scene = loadObjFolderBuildGL(objDir);
        } catch (const std::exception& e) {
            std::cerr << "OBJ loading error: " << e.what() << std::endl;
            return -1;
        }
        if (scene.meshes.empty()) {
            std::cout << "No .obj found in folder: " << objDir << "\n";
            return 0;
        }

        glViewport(0, 0, 640, 480);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

        while (!glfwWindowShouldClose(window))
        {
            glClear(GL_COLOR_BUFFER_BIT);
            glfwSwapBuffers(window);
            glfwPollEvents();
            glUseProgram(program);
            for (const auto& mesh : scene.meshes) {
                glBindVertexArray(mesh.vao);
                glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, 0);
            }
            glBindVertexArray(0);
        }

        glDeleteProgram(program);
        glfwTerminate();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    }
}