#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

void simulate(const char* filename) {
    mjModel* m = mj_loadXML(filename, nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Error loading model" << std::endl;
        return;
    }
    mjData* d = mj_makeData(m);
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
        mj_step(m, d);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
        return 1;
    }
    simulate(argv[1]);
    return 0;
}
