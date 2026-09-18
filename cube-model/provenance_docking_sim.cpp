#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>



// ------------------------------------------------------------------------------
// 1. BASIC 3D VECTOR MATH
// ------------------------------------------------------------------------------
struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 Cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y,
                    z * o.x - x * o.z,
                    x * o.y - y * o.x);
    }
    double Magnitude() const { return std::sqrt(Dot(*this)); }
};

// ------------------------------------------------------------------------------
// 2. QUATERNION 3D ORIENTATION (Section 1.6 of the Paper)
// Represents orientation as a 4D unit quaternion q = [w, (x, y, z)].
// Quaternions completely avoid "Gimbal Lock" (axis locking) that plagues Euler angles.
// The specific Hamilton-product / normalization / RPY-conversion code below is
// standard rigid-body math, not paper-specific, but is consistent with it.
// ------------------------------------------------------------------------------
struct Quat {
    double w, x, y, z;

    // Default: Identity quaternion (no rotation, w=1, x=0, y=0, z=0)
    Quat(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}

    // Quaternion Multiplication (Hamilton Product)
    // Combines two 3D rotations: (q_new * q_old)
    Quat operator*(const Quat& q) const {
        return Quat(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Normalizes quaternion to length = 1.0 to prevent numerical drift
    Quat Normalized() const {
        double mag = std::sqrt(w * w + x * x + y * y + z * z);
        if (mag < 1e-9) return Quat(1, 0, 0, 0);
        return Quat(w / mag, x / mag, y / mag, z / mag);
    }

    // Convert an angular rotation vector (axis * angle in radians) into a quaternion
    // For rotation vector d_theta: angle = ||d_theta||, axis = d_theta / angle
    static Quat FromAngularDisplacement(const Vec3& delta_theta) {
        double angle = delta_theta.Magnitude();
        if (angle < 1e-9) return Quat(1.0, 0.0, 0.0, 0.0);
        double halfAngle = angle * 0.5;
        double sinHalf = std::sin(halfAngle);
        return Quat(std::cos(halfAngle),
                    (delta_theta.x / angle) * sinHalf,
                    (delta_theta.y / angle) * sinHalf,
                    (delta_theta.z / angle) * sinHalf);
    }

    // Convert quaternion to a 4x4 rotation matrix for OpenGL rendering
    glm::mat4 ToRotationMatrix() const {
        glm::mat4 m(1.0f);
        m[0][0] = (float)(1.0 - 2.0 * (y * y + z * z));
        m[0][1] = (float)(2.0 * (x * y + w * z));
        m[0][2] = (float)(2.0 * (x * z - w * y));

        m[1][0] = (float)(2.0 * (x * y - w * z));
        m[1][1] = (float)(1.0 - 2.0 * (x * x + z * z));
        m[1][2] = (float)(2.0 * (y * z + w * x));

        m[2][0] = (float)(2.0 * (x * z + w * y));
        m[2][1] = (float)(2.0 * (y * z - w * x));
        m[2][2] = (float)(1.0 - 2.0 * (x * x + y * y));
        return m;
    }

    // Convert to Roll-Pitch-Yaw in degrees
    Vec3 ToRPYDegrees() const {
        // Roll (about X)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (about Y)
        double sinp = 2.0 * (w * y - z * x);
        double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

        // Yaw (about Z)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vec3(roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }
};

// ------------------------------------------------------------------------------
// 3. RIGID BODY STATE 
// [FROM THE PAPER: mass = 50 kg, inertia = diag(5, 5, 10) kg*m^2
// ------------------------------------------------------------------------------
struct RigidBody {
    std::string name;
    double mass;         // kg (Section 3.2: 50 kg)
    Vec3 inertia;        // Principal moments: diag(5, 5, 10) kg*m^2
    Vec3 position;       // Position (x, y, z) in meters
    Vec3 velocity;       // Linear velocity (vx, vy, vz) in m/s
    Quat attitude;       // Orientation quaternion [w, x, y, z]
    Vec3 angularVel;     // Angular velocity (wx, wy, wz) in rad/s
    Vec3 size;           // Dimensions (width, height, depth)
};

// ------------------------------------------------------------------------------
// 4. LOAD CELL LAYOUT 
// [FROM THE PAPER: 6 load cells on a regular hexagon, radius = 0.5 m]
// ------------------------------------------------------------------------------
const Vec3 g_loadCellPos[6] = {
    Vec3( 0.0,  0.50,  0.000), // Cell 1
    Vec3( 0.0,  0.25,  0.433), // Cell 2
    Vec3( 0.0, -0.25,  0.433), // Cell 3
    Vec3( 0.0, -0.50,  0.000), // Cell 4
    Vec3( 0.0, -0.25, -0.433), // Cell 5
    Vec3( 0.0,  0.25, -0.433)  // Cell 6
};

// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
void DistributeContactForceToLoadCells(const Vec3& totalContactForce, const Vec3& contactPt, Vec3 outForces[6]) {
    // Torque generated by contact force about the platform center
    Vec3 torque = contactPt.Cross(totalContactForce);

    // Sum of squared distances for load cell distribution
    double sumY2 = 0.0, sumZ2 = 0.0;
    for (int i = 0; i < 6; ++i) {
        sumY2 += g_loadCellPos[i].y * g_loadCellPos[i].y;
        sumZ2 += g_loadCellPos[i].z * g_loadCellPos[i].z;
    }

    for (int i = 0; i < 6; ++i) {
        // Normal force distribution (along X - contact normal)
        // F_x = Total_F_x / 6 + Moment_z * y_i / sum(y^2) - Moment_y * z_i / sum(z^2)
        double f_normal = (totalContactForce.x / 6.0) 
                        + (torque.z * g_loadCellPos[i].y / sumY2) 
                        - (torque.y * g_loadCellPos[i].z / sumZ2);

        // Shear components distribute equally
        double f_shear_y = totalContactForce.y / 6.0;
        double f_shear_z = totalContactForce.z / 6.0;

        outForces[i] = Vec3(f_normal, f_shear_y, f_shear_z);
    }
}

// ------------------------------------------------------------------------------
// 6. POST-CONTACT POSE ESTIMATION 
//   F_net = sum(F_i)
//   tau_net = sum(r_i x F_i)
//   P_c = (F_net x tau_net) / ||F_net||^2
//   a_CM = F_net / m
//   alpha = I^-1 * tau_net
//   Delta_x = 1/2 a_CM * (dt)^2
//   Delta_theta = 1/2 alpha * (dt)^2
// ------------------------------------------------------------------------------
struct ContactEstimationResult {
    Vec3 F_net;
    Vec3 tau_net;
    Vec3 Pc_est;
    Vec3 a_CM;
    Vec3 alpha;
    Vec3 delta_x;
    Vec3 delta_theta;
};

ContactEstimationResult ComputePostContactPose(const Vec3 forces[6], double mass, const Vec3& inertia, double dt) {
    ContactEstimationResult res;

    // Step 1: Net force vector 
    res.F_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i) {
        res.F_net = res.F_net + forces[i];
    }

    // Step 2: Net torque 
    res.tau_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i) {
        res.tau_net = res.tau_net + g_loadCellPos[i].Cross(forces[i]);
    }

    // Step 3: Contact Point Estimation P_c 
    double f_sq = res.F_net.Dot(res.F_net);
    if (f_sq > 1e-9) {
        res.Pc_est = res.F_net.Cross(res.tau_net) * (1.0 / f_sq);
    } else {
        res.Pc_est = Vec3(0, 0, 0);
    }

    // Step 4: Rigid Body Accelerations 
    res.a_CM = res.F_net * (1.0 / mass);
    res.alpha = Vec3(res.tau_net.x / inertia.x,
                     res.tau_net.y / inertia.y,
                     res.tau_net.z / inertia.z);

    // Step 5: Displacements over small interval dt (2 ms)
    double dt_sq_half = 0.5 * dt * dt;
    res.delta_x = res.a_CM * dt_sq_half;
    res.delta_theta = res.alpha * dt_sq_half;

    return res;
}

// ------------------------------------------------------------------------------
// 7. SHADERS & MESHES
// ------------------------------------------------------------------------------
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 FragPos;
out vec3 Normal;

void main() {
    FragPos = vec3(uModel * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;

out vec4 FragColor;

void main() {
    vec3 lightDir = normalize(vec3(2.0, 3.0, 2.5));
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 ambient = 0.35 * uColor;
    vec3 diffuse = diff * uColor * 0.85;
    FragColor = vec4(ambient + diffuse, 1.0);
}
)";

GLuint CreateShaderProgram() {
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertexShaderSource, NULL);
    glCompileShader(vs);

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragmentShaderSource, NULL);
    glCompileShader(fs);

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

void CreateCubeMesh(GLuint& vao, GLuint& vbo) {
    float vertices[] = {
        // positions          // normals
        -0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f,

        -0.5f, -0.5f,  0.5f,   0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,   0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,   0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,   0.0f,  0.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,   0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,   0.0f,  0.0f,  1.0f,

        -0.5f,  0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,

         0.5f,  0.5f,  0.5f,   1.0f,  0.0f,  0.0f,
         0.5f,  0.5f, -0.5f,   1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,   1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,   1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,   1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,   1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,   0.0f, -1.0f,  0.0f,
         0.5f, -0.5f, -0.5f,   0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,   0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,   0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,   0.0f, -1.0f,  0.0f,

        -0.5f,  0.5f, -0.5f,   0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,   0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,   0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,   0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,   0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,   0.0f,  1.0f,  0.0f
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

// ------------------------------------------------------------------------------
// 8. SIMULATION SCENARIO MANAGEMENT
// ------------------------------------------------------------------------------
enum class Scenario {
    HeadOn = 1,    // Scenario 1: Straight-line head-on collision (Zero rotation)
    OffCenter = 2  // Scenario 2: Off-center collision (Produces torque & spin)
};

Scenario g_currentScenario = Scenario::HeadOn;
RigidBody g_target, g_chaser;
bool g_contactOccurred = false;
bool g_paused = false;

void SetupScenario(Scenario sc) {
    g_currentScenario = sc;
    g_contactOccurred = false;

    // Target (Stationary at origin)
    g_target.name = "Target (ISS)";
    g_target.mass = 50.0;
    g_target.inertia = Vec3(5.0, 5.0, 10.0);
    g_target.position = Vec3(0.0, 0.0, 0.0);
    g_target.velocity = Vec3(0.0, 0.0, 0.0);
    g_target.attitude = Quat(1.0, 0.0, 0.0, 0.0); // Identity quaternion
    g_target.angularVel = Vec3(0.0, 0.0, 0.0);
    g_target.size = Vec3(1.0, 1.0, 1.0);

    // Chaser (Spacecraft approaching at 0.5 m/s)
    g_chaser.name = "Chaser";
    g_chaser.mass = 50.0;
    g_chaser.inertia = Vec3(5.0, 5.0, 10.0);
    g_chaser.velocity = Vec3(0.5, 0.0, 0.0); // Steady approach along +X
    g_chaser.attitude = Quat(1.0, 0.0, 0.0, 0.0);
    g_chaser.angularVel = Vec3(0.0, 0.0, 0.0);
    g_chaser.size = Vec3(1.0, 1.0, 1.0);

    if (sc == Scenario::HeadOn) {
        // Aligned along X axis: Head-on hit at center
        g_chaser.position = Vec3(-3.0, 0.0, 0.0);
        std::cout << "\n=======================================================\n";
        std::cout << " [ACTIVE SCENARIO 1: HEAD-ON COLLISION (STRAIGHT REBOUND)]\n";
        std::cout << " Alignment: Dead-Center | Expected Torque: ZERO | Spin: ZERO\n";
        std::cout << " Controls: Press [2] for Off-Center | [R] Reset | [SPACE] Pause\n";
        std::cout << "=======================================================\n" << std::endl;
    } else {
        // Offset laterally along Y by 0.35m: Hits corner/edge off-center
        g_chaser.position = Vec3(-3.0, 0.35, 0.15);
        std::cout << "\n=======================================================\n";
        std::cout << " [ACTIVE SCENARIO 2: OFF-CENTER COLLISION (TORQUE & SPIN)]\n";
        std::cout << " Alignment: Offset (Y=0.35m, Z=0.15m) | Contact Point Off-Center\n";
        std::cout << " Controls: Press [1] for Head-On | [R] Reset | [SPACE] Pause\n";
        std::cout << "=======================================================\n" << std::endl;
    }
}

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_1) {
            SetupScenario(Scenario::HeadOn);
        } else if (key == GLFW_KEY_2) {
            SetupScenario(Scenario::OffCenter);
        } else if (key == GLFW_KEY_R) {
            SetupScenario(g_currentScenario);
        } else if (key == GLFW_KEY_SPACE) {
            g_paused = !g_paused;
            std::cout << (g_paused ? ">>> [PAUSED]" : ">>> [RESUMED]") << std::endl;
        }
    }
}

// ------------------------------------------------------------------------------
// 9. MAIN EXECUTION
// ------------------------------------------------------------------------------
int main() {
    std::cout << "===============================================================\n";
    std::cout << " PHYSICS-BASED RIGID BODY DOCKING & COLLISION (PES Univ)\n";
    std::cout << " Contact reconstruction from Dr. Balasubramanyam's paper,\n";
    std::cout << " fed by an author-written synthetic load-cell forward model\n";
    std::cout << "===============================================================\n";

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(1100, 700, "Rigid Body Docking Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // VSync 60 FPS
    glfwSetKeyCallback(window, KeyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    GLuint shaderProgram = CreateShaderProgram();
    GLuint cubeVAO, cubeVBO;
    CreateCubeMesh(cubeVAO, cubeVBO);

    SetupScenario(Scenario::HeadOn);

    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        double currentTime = glfwGetTime();
        double frameDt = currentTime - lastTime;
        lastTime = currentTime;
        if (frameDt > 0.033) frameDt = 0.033;

        if (!g_paused) {
            // 3D Overlap / Contact Check between the two boxes
            double overlapX = (g_chaser.position.x + g_chaser.size.x * 0.5) - (g_target.position.x - g_target.size.x * 0.5);
            double overlapY = (g_chaser.size.y * 0.5 + g_target.size.y * 0.5) - std::abs(g_chaser.position.y - g_target.position.y);
            double overlapZ = (g_chaser.size.z * 0.5 + g_target.size.z * 0.5) - std::abs(g_chaser.position.z - g_target.position.z);

            bool isTouching = (overlapX >= 0.0) && (overlapY > 0.0) && (overlapZ > 0.0);

            if (!g_contactOccurred && isTouching) {
                g_contactOccurred = true;

                // 1. Determine exact physical contact point P_c on Target interface (X = -0.5)
                // purely to have something realistic to feed the synthetic sensor model.]
                double pc_y = (g_chaser.position.y + g_target.position.y) * 0.5;
                double pc_z = (g_chaser.position.z + g_target.position.z) * 0.5;
                Vec3 physicalContactPt(0.0, pc_y, pc_z);

                // 2. Physical Collision Impulse (Elastic restitution e = 0.85)
                // model, used only to generate a plausible contact force magnitude.]
                double e = 0.85;
                double v_rel = g_chaser.velocity.x - g_target.velocity.x;
                double reducedMass = (g_chaser.mass * g_target.mass) / (g_chaser.mass + g_target.mass);
                double totalImpulse = (1.0 + e) * reducedMass * v_rel; // N*s

                // Contact lasts over discrete window dt = 2 ms
                const double dt_contact = 0.002;
                double contactForceMagnitude = totalImpulse / dt_contact; // N

                // Total contact force vector pushing Target along +X
                Vec3 totalContactForce(contactForceMagnitude, 0.0, 0.0);

                // 3. SYNTHETIC SENSOR STEP
                Vec3 loadCellForces[6];
                DistributeContactForceToLoadCells(totalContactForce, physicalContactPt, loadCellForces);

                // 4. RECONSTRUCTION

                ContactEstimationResult est = ComputePostContactPose(loadCellForces, g_target.mass, g_target.inertia, dt_contact);

                // 5. Post-Contact Velocity Calculation via exact Newton-Euler Impulse
                // Turning that into ongoing velocities
                // for both bodies, including the chaser's reaction, is this demo's own
                // extension so the two cubes visibly separate after contact.]
                Vec3 delta_v_target = est.a_CM * dt_contact;
                Vec3 delta_w_target = est.alpha * dt_contact;

                // Update Target post-contact velocities
                g_target.velocity = g_target.velocity + delta_v_target;
                g_target.angularVel = g_target.angularVel + delta_w_target;

                // Newton's 3rd Law on Chaser: receives equal and opposite impulse
                Vec3 delta_v_chaser = est.a_CM * (-1.0 * (g_target.mass / g_chaser.mass) * dt_contact);
                Vec3 delta_w_chaser = est.alpha * (-0.5 * dt_contact);

                g_chaser.velocity = g_chaser.velocity + delta_v_chaser;
                g_chaser.angularVel = g_chaser.angularVel + delta_w_chaser;

                // Compute Quaternion representation of contact rotation
                Quat delta_q_contact = Quat::FromAngularDisplacement(est.delta_theta);
                Vec3 rpyDeg = delta_q_contact.ToRPYDegrees();

                // Terminal Telemetry Output
                std::cout << "\n===============================================================\n";
                std::cout << "                 [PHYSICAL CONTACT EVENT OCCURRED]              \n";
                std::cout << "===============================================================\n";
                std::cout << "Incoming Rel Velocity:   " << v_rel << " m/s\n";
                std::cout << "Collision Impulse:       " << totalImpulse << " N*s (over dt = 2 ms)\n";
                std::cout << "True Physical Contact Pt:(" << physicalContactPt.x << ", " << physicalContactPt.y << ", " << physicalContactPt.z << ") m [synthetic, not from paper]\n";
                std::cout << "---------------------------------------------------------------\n";
                std::cout << "6 LOAD CELL READINGS (synthetic, from our distribution model):\n";
                for (int i = 0; i < 6; ++i) {
                    std::cout << "  Cell " << (i+1) << ": (" 
                              << std::fixed << std::setprecision(1) << loadCellForces[i].x << ", " 
                              << loadCellForces[i].y << ", " << loadCellForces[i].z << ") N\n";
                }
                std::cout << "---------------------------------------------------------------\n";
                std::cout << "RECONSTRUCTION FROM PAPER FORMULAS (Section 2.5 - 2.6):\n";
                std::cout << "  Net Force F_net:       (" << est.F_net.x << ", " << est.F_net.y << ", " << est.F_net.z << ") N\n";
                std::cout << "  Net Torque tau_net:    (" << est.tau_net.x << ", " << est.tau_net.y << ", " << est.tau_net.z << ") N*m\n";
                std::cout << "  ESTIMATED CONTACT PT:  (" << est.Pc_est.x << ", " << est.Pc_est.y << ", " << est.Pc_est.z << ") m\n";
                std::cout << "  Linear Accel a_CM:     (" << est.a_CM.x << ", " << est.a_CM.y << ", " << est.a_CM.z << ") m/s^2\n";
                std::cout << "  Angular Accel alpha:   (" << est.alpha.x << ", " << est.alpha.y << ", " << est.alpha.z << ") rad/s^2\n";
                std::cout << "  Post-Contact Disp Delta_x: (" << est.delta_x.x << ", " << est.delta_x.y << ", " << est.delta_x.z << ") m\n";
                std::cout << "  QUATERNION ROTATION Delta_q: [" 
                          << std::setprecision(6) << delta_q_contact.w << ", " << delta_q_contact.x << ", " 
                          << delta_q_contact.y << ", " << delta_q_contact.z << "]\n";
                std::cout << "  Equivalent Euler RPY:  (" << rpyDeg.x << "°, " << rpyDeg.y << "°, " << rpyDeg.z << "°)\n";
                std::cout << "---------------------------------------------------------------\n";
                std::cout << "RESULTING MOTION (author's extension, not from paper):\n";
                std::cout << "  Target Linear Velocity: (" << g_target.velocity.x << ", " << g_target.velocity.y << ", " << g_target.velocity.z << ") m/s\n";
                std::cout << "  Target Angular Rate:    (" << g_target.angularVel.x << ", " << g_target.angularVel.y << ", " << g_target.angularVel.z << ") rad/s\n";
                std::cout << "  Chaser Linear Velocity: (" << g_chaser.velocity.x << ", " << g_chaser.velocity.y << ", " << g_chaser.velocity.z << ") m/s\n";
                std::cout << "===============================================================\n" << std::endl;
            }

            // Continuous motion in zero-gravity
            // Position update: x(t+dt) = x(t) + v * dt
            g_target.position = g_target.position + (g_target.velocity * frameDt);

            // Quaternion Orientation update:
            // delta_q = FromAngularDisplacement(omega * dt)
            // q(t+dt) = (delta_q * q(t)).Normalized()
            Quat delta_q_target = Quat::FromAngularDisplacement(g_target.angularVel * frameDt);
            g_target.attitude = (delta_q_target * g_target.attitude).Normalized();

            g_chaser.position = g_chaser.position + (g_chaser.velocity * frameDt);
            Quat delta_q_chaser = Quat::FromAngularDisplacement(g_chaser.angularVel * frameDt);
            g_chaser.attitude = (delta_q_chaser * g_chaser.attitude).Normalized();
        }

        // ----------------------------------------------------------------------
        // RENDER SCENE
        // ----------------------------------------------------------------------
        glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        // Perspective camera
        glm::mat4 projection = glm::perspective(glm::radians(42.0f), 1100.0f / 700.0f, 0.1f, 50.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 3.8f, 6.8f), glm::vec3(0.0f, 0.2f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

        glBindVertexArray(cubeVAO);

        // --- Render Target Cube (Cyan) ---
        // Model Matrix = Translation * Quaternion_Rotation_Matrix * Scale
        glm::mat4 modelTarget = glm::translate(glm::mat4(1.0f), glm::vec3(g_target.position.x, g_target.position.y, g_target.position.z));
        modelTarget = modelTarget * g_target.attitude.ToRotationMatrix();
        modelTarget = glm::scale(modelTarget, glm::vec3(g_target.size.x, g_target.size.y, g_target.size.z));

        glm::mat4 mvpTarget = projection * view * modelTarget;
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(mvpTarget));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(modelTarget));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.25f, 0.70f, 0.90f); // Cyan
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // --- Render Chaser Cube (Coral) ---
        glm::mat4 modelChaser = glm::translate(glm::mat4(1.0f), glm::vec3(g_chaser.position.x, g_chaser.position.y, g_chaser.position.z));
        modelChaser = modelChaser * g_chaser.attitude.ToRotationMatrix();
        modelChaser = glm::scale(modelChaser, glm::vec3(g_chaser.size.x, g_chaser.size.y, g_chaser.size.z));

        glm::mat4 mvpChaser = projection * view * modelChaser;
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(mvpChaser));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(modelChaser));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.95f, 0.45f, 0.25f); // Coral
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // --- Reference Track ---
        glm::mat4 modelFloor = glm::mat4(1.0f);
        modelFloor = glm::translate(modelFloor, glm::vec3(0.0f, -0.65f, 0.0f));
        modelFloor = glm::scale(modelFloor, glm::vec3(12.0f, 0.02f, 3.0f));
        glm::mat4 mvpFloor = projection * view * modelFloor;
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(mvpFloor));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(modelFloor));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.18f, 0.20f, 0.25f);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(shaderProgram);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
