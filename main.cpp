#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

// グローバル変数
mjModel* m = nullptr;        // MuJoCo model
mjData* d = nullptr;         // MuJoCo data
mjvCamera cam;               // カメラ
mjvOption opt;               // オプション
mjvScene scn;                // シーン
mjrContext con;              // コンテキスト

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// マウスボタンコールバック
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update last click position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// カーソル位置コールバック
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // determine action based on mouse button
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// スクロールコールバック
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// キーコールバック
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_R) {
            // シミュレーションをリセット
            mj_resetData(m, d);
        }
    }
}

void print_contact_forces(){
    // 現在の接触数を取得
    int ncon = d->ncon;

    // 各接触についてループ
    for (int i = 0; i < ncon; i++){
        // i番目の接触を取得
        mjContact* contact = &d->contact[i];

        // 接触力を取得
        mjtNum force[6];
        mju_zero(force, 6);
        mj_contactForce(m, d, i, force);

        // 接触フレーム：法線はz軸、接線はxとy
        // force[0]とforce[1]は接線方向の力
        // force[2]は法線方向の力
        //std::cout << "Contact " << i << ": Force = (" 
        //<< force[0] << ", " << force[1] << ", " << force[2] << ")" << std::endl;

        // 接触法線をグローバルフレームで取得
        mjtNum normal[3];
        normal[0] = contact->frame[0];
        normal[1] = contact->frame[1];
        normal[2] = contact->frame[2];

        //std::cout << "Contact " << i << ": Normal = ("
        //<< normal[0] << ", " << normal[1] << ", " << normal[2] << ")" << std::endl;

        // 法線方向の力をグローバルフレームに変換
        mjtNum global_force[3];
        global_force[0] = force[0] * normal[0];
        global_force[1] = force[0] * normal[1];
        global_force[2] = force[0] * normal[2];

        std::cout << "Contact " << i << ": Force = ("
        << global_force[0] << ", " << global_force[1] << ", " << global_force[2] << ")" << std::endl;
    }
}

void simulate(const char* filename) {
    // モデルを読み込む
    m = mj_loadXML(filename, nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Error loading model" << std::endl;
        return;
    }
    d = mj_makeData(m);

    // GLFWの初期化
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        mj_deleteModel(m); // モデルをクリーンアップ
        return;
    }

    // ウィンドウの作成
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        mj_deleteData(d);  // データをクリーンアップ
        mj_deleteModel(m); // モデルをクリーンアップ
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // MuJoCoの設定を初期化
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 視覚化オプションを設定
    opt.flags[mjVIS_CONTACTPOINT] = 1;
    opt.flags[mjVIS_CONTACTFORCE] = 1;
    opt.flags[mjVIS_CONSTRAINT] = 1;

    // コールバック関数を設定
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key_callback);

    // メインループ
    while (!glfwWindowShouldClose(window)) {
        mj_step(m, d);  // シミュレーションを1ステップ進める
        print_contact_forces();  // 接触力を出力

        // ビューポートを設定
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // シーンを更新してレンダリング
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // バッファのスワップとイベントのポーリング
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // リソースを解放
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwDestroyWindow(window);
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
