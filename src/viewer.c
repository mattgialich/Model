/* viewer.c — optional raylib 3D viewer. The simulation core never depends on
 * this; it is a window that watches a World.
 *
 * Keys: 1/2/3 switch robot model, R reset, mouse drag orbits, wheel zooms.
 *
 * Coordinate note: the sim is Z-up, raylib is Y-up. The mapping
 * (x, y, z)_sim -> (x, z, -y)_raylib is a pure rotation, so quaternions map
 * the same way on their vector part.
 */
#include "robots.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdio.h>

static Vector3 to_rl(Vec3 v) {
    return (Vector3){(float)v.x, (float)v.z, (float)-v.y};
}
static Quaternion to_rlq(Quat q) {
    return (Quaternion){(float)q.x, (float)q.z, (float)-q.y, (float)q.w};
}

static const Color PALETTE[] = {
    {230, 126, 34, 255},  /* orange  */
    {52, 152, 219, 255},  /* blue    */
    {46, 204, 113, 255},  /* green   */
    {155, 89, 182, 255},  /* purple  */
    {241, 196, 15, 255},  /* yellow  */
    {26, 188, 156, 255},  /* teal    */
};

static void draw_link(const Robot *r, int i) {
    const Link *L = &r->links[i];
    Vec3 p = r->link_pos[i];
    Quat q = r->link_rot[i];
    Color col = L->joint == JOINT_WHEEL
              ? (Color){60, 60, 60, 255}
              : PALETTE[i % (int)(sizeof PALETTE / sizeof PALETTE[0])];

    switch (L->geom) {
    case GEOM_BOX: {
        rlPushMatrix();
        Vector3 pos = to_rl(p);
        rlTranslatef(pos.x, pos.y, pos.z);
        Matrix rot = QuaternionToMatrix(to_rlq(q));
        rlMultMatrixf(MatrixToFloat(rot));
        Vector3 off = to_rl(L->geom_offset);
        rlTranslatef(off.x, off.y, off.z);
        DrawCube((Vector3){0, 0, 0},
                 (float)L->dims.x, (float)L->dims.z, (float)L->dims.y, col);
        DrawCubeWires((Vector3){0, 0, 0},
                 (float)L->dims.x, (float)L->dims.z, (float)L->dims.y,
                 ColorBrightness(col, -0.4f));
        rlPopMatrix();
        break;
    }
    case GEOM_CYLINDER: {
        /* cylinder axis is local Y; compute world endpoints directly */
        Vec3 center = v3_add(p, quat_rotate(q, L->geom_offset));
        Vec3 axis   = quat_rotate(q, v3(0, 1, 0));
        Vec3 h      = v3_scale(axis, L->dims.y * 0.5);
        DrawCylinderEx(to_rl(v3_sub(center, h)), to_rl(v3_add(center, h)),
                       (float)L->dims.x, (float)L->dims.x, 20, col);
        DrawCylinderWiresEx(to_rl(v3_sub(center, h)), to_rl(v3_add(center, h)),
                       (float)L->dims.x, (float)L->dims.x, 20,
                       ColorBrightness(col, -0.4f));
        break;
    }
    case GEOM_SPHERE: {
        Vec3 center = v3_add(p, quat_rotate(q, L->geom_offset));
        DrawSphere(to_rl(center), (float)L->dims.x, col);
        break;
    }
    }
}

int main(int argc, char **argv) {
    int model_idx = 0;
    if (argc > 1) {
        const RobotModel *m = robot_model_find(argv[1]);
        if (m) model_idx = (int)(m - ROBOT_MODELS);
        else   fprintf(stderr, "unknown robot '%s', using %s\n",
                       argv[1], ROBOT_MODELS[0].name);
    }

    World w;
    world_init(&w, 0.001);
    world_load_model(&w, &ROBOT_MODELS[model_idx]);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "Model — robotics world");
    SetTargetFPS(60);

    float cam_yaw = 0.8f, cam_pitch = 0.35f, cam_dist = 2.2f;
    Camera3D cam = {0};
    cam.up         = (Vector3){0, 1, 0};
    cam.fovy       = 50.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    double accumulator = 0.0;

    while (!WindowShouldClose()) {
        /* --- input --- */
        for (int k = 0; k < N_ROBOT_MODELS && k < 9; k++) {
            if (IsKeyPressed(KEY_ONE + k)) {
                model_idx = k;
                world_load_model(&w, &ROBOT_MODELS[model_idx]);
            }
        }
        if (IsKeyPressed(KEY_R))
            world_load_model(&w, &ROBOT_MODELS[model_idx]);

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 d = GetMouseDelta();
            cam_yaw   += d.x * 0.006f;
            cam_pitch  = Clamp(cam_pitch + d.y * 0.006f, -0.1f, 1.4f);
        }
        cam_dist = Clamp(cam_dist - GetMouseWheelMove() * 0.2f, 0.5f, 12.0f);

        /* --- fixed-timestep sim --- */
        accumulator += GetFrameTime();
        if (accumulator > 0.1) accumulator = 0.1;   /* avoid spiral of death */
        while (accumulator >= w.dt) {
            world_step(&w);
            accumulator -= w.dt;
        }

        /* --- camera follows the robot base --- */
        Vector3 target = to_rl(w.robot.base_pos);
        target.y += 0.15f;
        cam.target   = target;
        cam.position = (Vector3){
            target.x + cam_dist * cosf(cam_pitch) * cosf(cam_yaw),
            target.y + cam_dist * sinf(cam_pitch),
            target.z + cam_dist * cosf(cam_pitch) * sinf(cam_yaw),
        };

        /* --- draw --- */
        BeginDrawing();
        ClearBackground((Color){24, 26, 30, 255});

        BeginMode3D(cam);
        DrawPlane((Vector3){target.x, -0.001f, target.z},
                  (Vector2){60, 60}, (Color){44, 48, 54, 255});
        rlPushMatrix();
        /* keep grid lines anchored to the world, snapped to 0.5 m cells */
        rlTranslatef(floorf(target.x * 2) / 2, 0, floorf(target.z * 2) / 2);
        DrawGrid(80, 0.5f);
        rlPopMatrix();

        for (int i = 0; i < w.robot.n_links; i++)
            draw_link(&w.robot, i);
        EndMode3D();

        DrawText(TextFormat("%s  |  %d links, %d actuators  |  t = %.1fs",
                            w.robot.name, w.robot.n_links,
                            w.robot.n_actuators, w.time),
                 14, 12, 20, RAYWHITE);
        for (int k = 0; k < N_ROBOT_MODELS; k++)
            DrawText(TextFormat("%d: %s", k + 1, ROBOT_MODELS[k].name),
                     14, 44 + 22 * k, 18,
                     k == model_idx ? (Color){230, 126, 34, 255} : GRAY);
        DrawText("R: reset   drag: orbit   wheel: zoom",
                 14, 44 + 22 * N_ROBOT_MODELS + 8, 18, GRAY);
        DrawFPS(GetScreenWidth() - 100, 12);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
