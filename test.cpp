#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <chrono>

// グローバル変数
mjModel* m = nullptr;        // MuJoCo model
mjData* d = nullptr;         // MuJoCo data
mjvCamera cam;               // カメラ
mjvOption opt;               // オプション
mjvScene scn;                // シーン
mjrContext con;              // コンテキスト
mjvPerturb pert;             // 摂動
mjvSceneState scnstate;      // シーン状態

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
bool dragging = false;       // ドラッグ状態を保持
int selected_geom = -1;      // 選択されたジオメトリID
int selected_body = -1;      // 選択されたボディID
bool is_perturb = false;     // 物体の選択状態を保持
mjtNum initial_pos[3];       // 選択時の初期位置
std::chrono::time_point<std::chrono::steady_clock> last_click_time; // 最後のクリック時間

// マウスボタンコールバック
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update last click position
    glfwGetCursorPos(window, &lastx, &lasty);

    if (act == GLFW_PRESS && button_left) {
        auto now = std::chrono::steady_clock::now();
        auto duration_since_last_click = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_click_time).count();

        // ダブルクリックを検出
        if (duration_since_last_click < 500) { // 500ミリ秒以内のクリックをダブルクリックと判定
            // レイキャスティングを行い、選択されたジオメトリIDを取得
            mjtNum selpnt[3];
            int selgeom, selbody, selfeature, selskin;
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            selbody = mjv_select(m, d, &opt, static_cast<mjtNum>(width) / height,
                                lastx / width, (height - lasty) / height, &scn, selpnt, &selgeom, &selfeature, &selskin);

            if (selbody >= 0) {
                if (selected_geom == selgeom) {
                    // 選択解除
                    selected_geom = -1;
                    selected_body = -1;
                    is_perturb = false;
                } else {
                    // 新しい物体を選択
                    is_perturb = true;
                    dragging = false; // ドラッグはまだ開始しない
                    selected_geom = selgeom; // 選択されたジオメトリIDを保存
                    selected_body = selbody; // 選択されたボディIDを保存
                    // 摂動の初期化
                    pert.active = mjPERT_TRANSLATE;
                    pert.select = selbody;
                    mju_copy3(pert.localpos, selpnt);

                    // 初期位置の保存
                    mju_copy3(initial_pos, d->xpos + 3 * selbody);
                }
            }
        }

        last_click_time = now; // 最後のクリック時間を更新
    } else if (act == GLFW_RELEASE) {
        dragging = false; // ドラッグを終了
    }
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
    } else if (button_middle || (button_left && shift)) {
        action = mjMOUSE_MOVE_H;
    } else if (button_left && !is_perturb) {
        action = mjMOUSE_ROTATE_H;
    } else if (button_left && is_perturb) {
        action = mjMOUSE_MOVE_V;
        dragging = true;
    } else {
        action = mjMOUSE_ZOOM;
    }

    if (dragging && selected_geom >= 0 && is_perturb) {
        // move perturb object
        mjv_movePerturb(m, d, action, dx / height, dy / height, &scn, &pert);
        // 相対位置の計算
        mjtNum displacement[3];
        mju_addToScl3(displacement, pert.localpos, dx / height);
        mju_addToScl3(displacement, pert.localpos, dy / height);
        // 新しい位置を計算
        mjtNum new_pos[3];
        mju_add3(new_pos, initial_pos, displacement);
        // 新しい位置を設定
        mju_copy3(d->xpos + 3 * selected_body, new_pos);
        mju_copy3(d->xipos + 3 * selected_body, new_pos);
    } else {
        // move camera
        mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
    }
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

void simulate(const char* filename) {
    m = mj_loadXML(filename, nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Error loading model" << std::endl;
        return;
    }
    d = mj_makeData(m);
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
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // シーン状態の初期化
    mjv_defaultSceneState(&scnstate);
    mjv_makeSceneState(m, d, &scnstate, 2000);

    // 摂動の初期化
    mjv_defaultPerturb(&pert);

    // コールバック関数を設定
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key_callback);

    while (!glfwWindowShouldClose(window)) {
        mj_step(m, d);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        // 選択された物体の色を更新
        for (int i = 0; i < scn.ngeom; ++i) {
            if (i == selected_geom) {
                scn.geoms[i].rgba[0] = 1.0f;  // 赤色
                scn.geoms[i].rgba[1] = 0.0f;
                scn.geoms[i].rgba[2] = 0.0f;
                scn.geoms[i].rgba[3] = 1.0f;
            } else {
                // 元の色に戻す
                scn.geoms[i].rgba[0] = 0.2f;  // 元の色 (例: 緑色)
                scn.geoms[i].rgba[1] = 0.7f;
                scn.geoms[i].rgba[2] = 0.2f;
                scn.geoms[i].rgba[3] = 1.0f;
            }
        }

        // 摂動の適用
        if (selected_body >= 0) {
            mjv_applyPerturbPose(m, d, &pert, 1);  // モーションキャプチャと動的ボディに適用
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjv_freeSceneState(&scnstate); // シーン状態を解放
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
