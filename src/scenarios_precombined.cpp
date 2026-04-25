/*
 * scenarios_precombined.cpp  (Enhanced Version with Dual Camera & CSV Logging)
 * Updated for MarsProjectStewartPlatform - self-contained paths.
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <ctime>

// === Added for simulation ===
#include "DockingSimulation.h"
#include <memory>
// ==========================

static const int SCR_W = 1280, SCR_H = 720;
static const int N_LEGS = 6;
static float SCALE = 0.005f;

static const float L_MIN = 0.15f; 
static const float L_MAX = 0.258f; 

static const char* PATHS[N_LEGS] = {
    "assets/models/leg_1.obj", "assets/models/leg_2.obj",
    "assets/models/leg_3.obj", "assets/models/leg_4.obj",
    "assets/models/leg_5.obj", "assets/models/leg_6.obj",
};
static const char* MOD_PLATE = "assets/models/bottomStewart.obj";
static const char* MOD_TARGET = "assets/models/Up_stewartPlatform.obj";
static const char* MOD_RIG = "assets/models/the_base.obj";

// === Scenario Paths ===
static const char* SCENARIO_NORMAL = "scenarios/normal_docking_new.xml";
static const char* SCENARIO_OFF_CENTER = "scenarios/offcenter_docking_new.xml";
// ======================

#include <filesystem>
std::string findRuntimePath(const std::string& path) {
    namespace fs = std::filesystem;
    const std::vector<fs::path> candidates = {
        fs::path(path),
        fs::path("..") / path,
        fs::path("../..") / path,
        fs::path("../../..") / path
    };
    for (const auto& candidate : candidates) {
        if (fs::exists(candidate)) return candidate.string();
    }
    return path;
}

struct Mesh { GLuint vao=0,vbo=0; int count=0; float width=0; };
Mesh upload(const std::vector<float>& v, float w) {
    Mesh m; m.count=(int)v.size()/6; m.width=w;
    glGenVertexArrays(1,&m.vao); glGenBuffers(1,&m.vbo);
    glBindVertexArray(m.vao); glBindBuffer(GL_ARRAY_BUFFER,m.vbo);
    glBufferData(GL_ARRAY_BUFFER,v.size()*4,v.data(),GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,0,24,0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1,3,GL_FLOAT,0,24,(void*)12);
    return m;
}

struct Model { 
    Mesh casing, rod; 
    glm::vec3 bMin, bMax, jb_local, jt_local;
    bool hasTwo = false; bool ok=false;
};

Model loadLeg(const std::string& path) {
    Assimp::Importer imp;
    const aiScene* sc = imp.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_PreTransformVertices);
    if (!sc) return {};
    Model res; glm::vec3 gMin(1e9), gMax(-1e9); std::vector<glm::vec3> all_vv;
    std::vector<Mesh> tempParts;
    for (unsigned mi=0; mi<sc->mNumMeshes; ++mi) {
        const aiMesh* am = sc->mMeshes[mi]; std::vector<float> buf;
        glm::vec3 pMin(1e9), pMax(-1e9);
        for (unsigned vi=0; vi<am->mNumVertices; ++vi) {
            glm::vec3 p = {am->mVertices[vi].x, am->mVertices[vi].y, am->mVertices[vi].z};
            all_vv.push_back(p); gMin = glm::min(gMin, p); gMax = glm::max(gMax, p);
            pMin = glm::min(pMin, p); pMax = glm::max(pMax, p);
            buf.push_back(p.x); buf.push_back(p.y); buf.push_back(p.z);
            if(am->HasNormals()){buf.push_back(am->mNormals[vi].x);buf.push_back(am->mNormals[vi].y);buf.push_back(am->mNormals[vi].z);}
            else{buf.push_back(0);buf.push_back(1);buf.push_back(0);}
        }
        float w = glm::length(glm::vec2(pMax.x-pMin.x, pMax.z-pMin.z));
        tempParts.push_back(upload(buf, w));
    }
    if(tempParts.size() >= 2) {
        std::sort(tempParts.begin(), tempParts.end(), [](const Mesh& a, const Mesh& b){ return a.width > b.width; });
        res.casing = tempParts[0]; res.rod = tempParts[1]; res.hasTwo = true;
    } else if(tempParts.size() == 1) { res.casing = tempParts[0]; }
    float eps = (gMax.y-gMin.y)*0.08f; glm::vec3 tA(0), bA(0); int tC=0, bC=0;
    for(const auto& v:all_vv){ if(v.y>gMax.y-eps){tA+=v;tC++;} if(v.y<gMin.y+eps){bA+=v;bC++;} }
    res.bMin=gMin; res.bMax=gMax; res.jt_local=(tC>0)?tA/(float)tC:gMax; res.jb_local=(bC>0)?bA/(float)bC:gMin;
    res.ok=true; return res;
}

Model loadSimple(const std::string& path) {
    Assimp::Importer imp; const aiScene* sc = imp.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_PreTransformVertices);
    if (!sc) return {}; Model res; glm::vec3 gMin(1e9), gMax(-1e9); std::vector<float> buf;
    for (unsigned mi=0; mi<sc->mNumMeshes; ++mi) {
        const aiMesh* am = sc->mMeshes[mi];
        for (unsigned vi=0; vi<am->mNumVertices; ++vi) {
            glm::vec3 p = {am->mVertices[vi].x, am->mVertices[vi].y, am->mVertices[vi].z};
            gMin = glm::min(gMin, p); gMax = glm::max(gMax, p);
            buf.push_back(p.x); buf.push_back(p.y); buf.push_back(p.z);
            if(am->HasNormals()){buf.push_back(am->mNormals[vi].x);buf.push_back(am->mNormals[vi].y);buf.push_back(am->mNormals[vi].z);}
            else{buf.push_back(0);buf.push_back(1);buf.push_back(0);}
        }
    }
    res.casing = upload(buf, 0); res.bMin=gMin; res.bMax=gMax; res.ok=true; return res;
}

Mesh sphere(float r){
    std::vector<float> b; int st=12, sl=18;
    for(int i=0; i<st; ++i){
        float p0=glm::pi<float>()*i/st, p1=glm::pi<float>()*(i+1)/st;
        for(int j=0; j<sl; ++j){
            float t0=2.f*glm::pi<float>()*j/sl, t1=2.f*glm::pi<float>()*(j+1)/sl;
            auto a=[&](float p,float t){ glm::vec3 n={sinf(p)*cosf(t),cosf(p),sinf(p)*sinf(t)}; b.push_back(n.x*r);b.push_back(n.y*r);b.push_back(n.z*r); b.push_back(n.x);b.push_back(n.y);b.push_back(n.z); };
            a(p0,t0); a(p1,t0); a(p1,t1); a(p0,t0); a(p1,t1); a(p0,t1);
        }
    }
    return upload(b, 0);
}

struct Cam {
    float yaw=45, pitch=25, dist=0.6f; glm::vec3 tgt={0,0.1,0};
    glm::vec3 pos()const{ float y=glm::radians(yaw),p=glm::radians(pitch); return tgt+dist*glm::vec3(cosf(p)*sinf(y),sinf(p),cosf(p)*cosf(y)); }
    glm::mat4 view()const{ return glm::lookAt(pos(),tgt,glm::vec3(0,1,0)); }
} g_cam;

int g_camMode = 0;
bool g_splitScreen = false;
struct TopCam {
    glm::vec3 pos()const{ return glm::vec3(0.0f, 0.13f, 0.01f); } 
    glm::vec3 tgt()const{ return glm::vec3(0.0f, 0.05f, 0.0f); }
    glm::mat4 view()const{ return glm::lookAt(pos(), tgt(), glm::vec3(0.0f, 1.0f, 0.0f)); }
} g_topCam;

struct Pose { glm::vec3 p; glm::vec3 r; } g_p, g_def = {{0,0.025,0}, {0,0,0}}, g_test;
static double mx, my; static bool md=false;

// === Added Scenario Engine Globals and Functions ===
std::unique_ptr<DockingSimulation> g_dockingSim;
bool g_playScenario = false;
double g_simAccumulator = 0.0;
std::string g_loadedScenarioLabel = "Manual";
Pose g_basePoseDisplay = {};
Pose g_suspendedPoseDisplay = {};

Pose toViewerPose(const DockingPose& pose) {
    Pose viewerPose;
    viewerPose.p = glm::vec3(
        static_cast<float>(pose.x),
        g_def.p.y + static_cast<float>(pose.z),
        static_cast<float>(pose.y));
    viewerPose.r = glm::degrees(glm::vec3(
        static_cast<float>(pose.roll),
        static_cast<float>(pose.pitch),
        static_cast<float>(pose.yaw)));
    return viewerPose;
}

void syncViewerPoseFromSimulation() {
    if (!g_dockingSim) return;

    g_p = toViewerPose(g_dockingSim->getCombinedPose());
    g_basePoseDisplay = toViewerPose(g_dockingSim->getBaseTargetPose());
    g_suspendedPoseDisplay = toViewerPose(g_dockingSim->getSuspendedTargetPose());
}

void loadScenario(const std::string& scenarioPath, const std::string& label) {
    const std::string resolvedPath = findRuntimePath(scenarioPath);

    try {
        g_dockingSim = std::make_unique<DockingSimulation>(resolvedPath);
        g_playScenario = false;
        g_simAccumulator = 0.0;
        g_loadedScenarioLabel = label;
        g_basePoseDisplay = {};
        g_suspendedPoseDisplay = {};
        syncViewerPoseFromSimulation();
        printf("[SCENARIO] Loaded %s from %s\n", label.c_str(), resolvedPath.c_str());
        printf("[SCENARIO] Press P to play/pause composed docking motion.\n");
    } catch (const std::exception& e) {
        g_dockingSim.reset();
        g_playScenario = false;
        printf("[SCENARIO] Failed to load %s: %s\n", label.c_str(), e.what());
    }
}

void updateScenarioPlayback(double dt) {
    if (!g_playScenario || !g_dockingSim) return;

    const double stepSize = g_dockingSim->getScenario()->config.timestep;
    g_simAccumulator += dt;

    while (g_simAccumulator >= stepSize) {
        g_dockingSim->step();
        g_simAccumulator -= stepSize;
    }

    syncViewerPoseFromSimulation();
}
// ===================================================

void onBtn(GLFWwindow*,int b,int a,int){ if(b==GLFW_MOUSE_BUTTON_LEFT)md=(a==GLFW_PRESS); }
void onPos(GLFWwindow*,double dx,double dy){ if(md){g_cam.yaw-=(float)(dx-mx)*.4f;g_cam.pitch+=(float)(dy-my)*.4f;g_cam.pitch=glm::clamp(g_cam.pitch,-89.f,89.f);} mx=dx;my=dy; }
void onScr(GLFWwindow*,double,double d){ g_cam.dist=glm::clamp(g_cam.dist-(float)d*.05f,0.05f,5.f); }

std::vector<glm::vec3> g_pBase, g_pTopHome;
bool validate(Pose pose){
    glm::mat4 T = glm::translate(glm::mat4(1.f), pose.p) * glm::toMat4(glm::quat(glm::radians(pose.r)));
    for(int i=0; i<N_LEGS; ++i) {
        float L = glm::distance(glm::vec3(T * glm::vec4(g_pTopHome[i], 1.f)), g_pBase[i]);
        if(L < L_MIN || L > L_MAX) return false;
    }
    return true;
}

void onKey(GLFWwindow*, int k, int, int a, int){
    if(a==GLFW_RELEASE)return; float s=0.006f, r=1.5f;

    // === Simulation Mode Controls ===
    if(k==GLFW_KEY_N) { loadScenario(SCENARIO_NORMAL, "Normal Docking"); return; }
    if(k==GLFW_KEY_O) { loadScenario(SCENARIO_OFF_CENTER, "Off-Center Contact"); return; }
    
    // === Camera Controls ===
    if(k==GLFW_KEY_C) { 
        g_camMode = (g_camMode + 1) % 2;
        printf("[CAM] Switched to %s camera view\n", g_camMode == 0 ? "DEFAULT" : "TOP-DOWN"); 
        return; 
    }
    if(k==GLFW_KEY_Z) { // Changed toggle to Z to avoid conflict with D (manual X control)
        g_splitScreen = !g_splitScreen;
        printf("[CAM] Split Screen %s\n", g_splitScreen ? "ON" : "OFF");
        return;
    }

    if(k==GLFW_KEY_P) {
        if(!g_dockingSim) { printf("[SCENARIO] Load a scenario first with N or O.\n"); return; }
        g_playScenario = !g_playScenario;
        printf("[SCENARIO] Playback %s (%s)\n", g_playScenario ? "ON" : "OFF", g_loadedScenarioLabel.c_str());
        return;
    }

    // === Manual Movement Controls ===
    g_test = g_p;
    if(k==GLFW_KEY_A || k==GLFW_KEY_LEFT) g_test.p.x-=s;
    if(k==GLFW_KEY_D || k==GLFW_KEY_RIGHT) g_test.p.x+=s;
    if(k==GLFW_KEY_UP) g_test.p.y+=s;
    if(k==GLFW_KEY_DOWN) g_test.p.y-=s;
    if(k==GLFW_KEY_PAGE_UP) g_test.p.z+=s;
    if(k==GLFW_KEY_PAGE_DOWN) g_test.p.z-=s;
    if(k==GLFW_KEY_W) g_test.r.x+=r; if(k==GLFW_KEY_S) g_test.r.x-=r;
    if(k==GLFW_KEY_Q) g_test.r.y+=r; if(k==GLFW_KEY_E) g_test.r.y-=r;
    if(k==GLFW_KEY_I) g_test.r.z+=r; if(k==GLFW_KEY_U) g_test.r.z-=r; // Use U instead of O for Yaw to avoid Scenario conflict
    
    if(k==GLFW_KEY_R) {
        g_p=g_def;
        g_playScenario=false;
        if (g_dockingSim) syncViewerPoseFromSimulation();
        printf("[LOG] RESET POSE\n");
        return;
    }
    
    if(g_playScenario) return; // Block manual if playing
    
    if(validate(g_test)) { 
        g_p = g_test; 
    } else { 
        if (std::abs(g_p.p.x) < 0.005f && g_p.p.y >= 0.060f && std::abs(g_p.p.z) < 0.005f && 
            std::abs(g_p.r.x) < 0.5f && std::abs(g_p.r.y) < 0.5f && std::abs(g_p.r.z) < 0.5f) {
            printf("successfully docked\n");
        } else {
            printf("Actuator contraint reached\n"); 
        }
    }
}

int main(){
    if(!glfwInit())return -1;
    glfwWindowHint(GLFW_SAMPLES,4);
    GLFWwindow* win=glfwCreateWindow(SCR_W,SCR_H,"Mars Project - Stewart Simulation (Cam v2)",0,0);
    glfwMakeContextCurrent(win); glfwSetMouseButtonCallback(win,onBtn); glfwSetCursorPosCallback(win,onPos); glfwSetScrollCallback(win,onScr); glfwSetKeyCallback(win,onKey);
    gladLoadGL(glfwGetProcAddress); glEnable(GL_DEPTH_TEST); auto c=[](GLenum t,const char* s){GLuint h=glCreateShader(t);glShaderSource(h,1,&s,0);glCompileShader(h);return h;};
    GLuint pr=glCreateProgram();
    glAttachShader(pr,c(GL_VERTEX_SHADER,R"(#version 330 core
layout(location=0)in vec3 p;layout(location=1)in vec3 n;
uniform mat4 uMVP; uniform mat4 uM; out vec3 vN,vP;
void main(){vP=(uM*vec4(p,1)).xyz; vN=(uM*vec4(n,0)).xyz; gl_Position=uMVP*vec4(p,1);})"));
    glAttachShader(pr,c(GL_FRAGMENT_SHADER,R"(#version 330 core
in vec3 vN,vP; uniform vec3 uCol; uniform vec3 uCam; uniform vec4 uClipPlane; out vec4 fC;
void main(){
    if(dot(vec4(vP,1.0), uClipPlane) < 0) discard;
    vec3 N=normalize(vN),L=normalize(uCam-vP);
    float d=max(dot(N,L),0.0)*0.6; fC=vec4(uCol*(0.4+d),1.0);
})"));
    glLinkProgram(pr);
    GLuint locClip = glGetUniformLocation(pr,"uClipPlane");
    GLuint locM = glGetUniformLocation(pr,"uM");

    Model platMod = loadSimple(findRuntimePath(MOD_PLATE));
    Model targetMod = loadSimple(findRuntimePath(MOD_TARGET));
    Model rigMod = loadSimple(findRuntimePath(MOD_RIG));
    Model lMods[N_LEGS]; for(int i=0;i<N_LEGS;++i) lMods[i]=loadLeg(findRuntimePath(PATHS[i]));
    Mesh sB=sphere(0.012f), sT=sphere(0.01f);
    glm::vec3 pltC = (platMod.bMin + platMod.bMax) * 0.5f;
    glm::mat4 v16WorldT = glm::scale(glm::mat4(1.f),glm::vec3(SCALE)) * glm::translate(glm::mat4(1.f),-pltC);

    float minY=1e9f;
    for(int i=0;i<N_LEGS;++i){
        g_pBase.push_back(glm::vec3(v16WorldT*glm::vec4(lMods[i].jb_local,1.f)));
        g_pTopHome.push_back(glm::vec3(v16WorldT*glm::vec4(lMods[i].jt_local,1.f)));
        minY = std::min(minY, g_pBase[i].y);
    }
    g_p = g_def; Pose lastP;
    glm::mat4 proj = glm::perspective(glm::radians(45.f),(float)SCR_W/SCR_H,0.01f,10.f);

    double lastFrameTime = glfwGetTime();

    // Startup Logs
    printf("\n========= MARS PROJECT STEWART SIMULATION (CAM V2) =========\n");
    printf("[N/O] Load Scenarios\n");
    printf("[C] Switch Camera | [Z] Toggle Split-Screen\n");
    printf("[P] Play/Pause | [R] Reset\n");
    printf("Manual: WASD (Pitch/X), QE (Roll), Arrows (X/Y), PgUp/Dn (Z)\n");
    printf("============================================================\n");

    // CSV Logging Setup
    std::time_t now = std::time(nullptr);
    char bufFileName[128];
    std::strftime(bufFileName, sizeof(bufFileName), "simulation_metrics_%Y%m%d_%H%M%S.csv", std::localtime(&now));
    std::ofstream csv(bufFileName);
    if(csv.is_open()) {
        csv << "Time,Pos_X,Pos_Y,Pos_Z,Roll,Pitch,Yaw,Leg_1,Leg_2,Leg_3,Leg_4,Leg_5,Leg_6\n";
    }

    while(!glfwWindowShouldClose(win)){
        const double currentFrameTime = glfwGetTime();
        const double dt = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        glfwPollEvents();
        updateScenarioPlayback(dt);

        // Kinematic Hard-Stop
        if (g_playScenario && !validate(g_p)) {
            bool overExtension = false;
            glm::mat4 testT = glm::translate(glm::mat4(1.f), g_p.p) * glm::toMat4(glm::quat(glm::radians(g_p.r)));
            for(int i=0; i<N_LEGS; ++i) {
                float L = glm::distance(glm::vec3(testT * glm::vec4(g_pTopHome[i], 1.f)), g_pBase[i]);
                if (L > L_MAX) overExtension = true;
            }
            if (overExtension) {
                g_p = lastP; g_playScenario = false;
                if (g_loadedScenarioLabel.find("Normal") != std::string::npos) {
                    printf("[STATUS] successfully docked\n[STATUS] docking finished\n");
                } else {
                    printf("[STATUS] unsuccessful docking - reach limit\n");
                }
            }
        }

        glClearColor(0.85, 0.86, 0.88, 1); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        float legLengths[N_LEGS];
        auto drawScene = [&](const glm::mat4& viewMat, const glm::vec3& camPos, int clipMode, const glm::mat4& projMat) {
            glm::mat4 VP = projMat * viewMat; 
            glUseProgram(pr);
            glUniform3fv(glGetUniformLocation(pr,"uCam"), 1, glm::value_ptr(camPos));
            glm::mat4 ctrlT = glm::translate(glm::mat4(1.f), g_p.p) * glm::toMat4(glm::quat(glm::radians(g_p.r)));
            
            // Render Rig
            glUniform4f(locClip, 0, 0, 0, 10000.f);
            glm::mat4 rigM = v16WorldT;
            glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*rigM));
            glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(rigM));
            glUniform3f(glGetUniformLocation(pr,"uCol"), 0.50, 0.52, 0.55); 
            glBindVertexArray(rigMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, rigMod.casing.count);

            // Render Plate
            if (clipMode != 0) glUniform4f(locClip, 0, 0, 0, 10000.f); 
            else glUniform4f(locClip, 0, 1, 0, -minY - 0.05f);
            glm::mat4 pM = ctrlT * v16WorldT;
            glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*pM));
            glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(pM));
            glUniform3f(glGetUniformLocation(pr,"uCol"), 0.72, 0.74, 0.76); 
            glBindVertexArray(platMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, platMod.casing.count);

            // Render Target
            glUniform4f(locClip, 0, 0, 0, 10000.f);
            glm::mat4 targetT = glm::translate(glm::mat4(1.f), {0, 0.0343f, 0}) * v16WorldT;
            glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*targetT));
            glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(targetT));
            glUniform3f(glGetUniformLocation(pr,"uCol"), 0.72, 0.74, 0.76); 
            glBindVertexArray(targetMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, targetMod.casing.count);

            for(int i=0; i<N_LEGS; ++i) {
                glm::vec3 B = g_pBase[i], T = glm::vec3(ctrlT * glm::vec4(g_pTopHome[i], 1.f)), vec = T-B;
                legLengths[i] = glm::length(vec);
                glm::quat rotQ = glm::rotation(glm::normalize(g_pTopHome[i]-g_pBase[i]), glm::normalize(vec));
                glm::mat4 rotM = glm::toMat4(rotQ);
                glm::mat4 matC = glm::translate(glm::mat4(1.f), B) * rotM * glm::translate(glm::mat4(1.f), -B) * v16WorldT;
                glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*matC)); 
                glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(matC));
                glUniform3f(glGetUniformLocation(pr,"uCol"), 0.58, 0.60, 0.62); 
                glBindVertexArray(lMods[i].casing.vao); glDrawArrays(GL_TRIANGLES, 0, lMods[i].casing.count);
                if(lMods[i].hasTwo) {
                    glm::mat4 matR = glm::translate(glm::mat4(1.f), T) * rotM * glm::translate(glm::mat4(1.f), -g_pTopHome[i]) * v16WorldT;
                    glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*matR)); 
                    glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(matR));
                    glUniform3f(glGetUniformLocation(pr,"uCol"), 0.88, 0.88, 0.92); 
                    glBindVertexArray(lMods[i].rod.vao); glDrawArrays(GL_TRIANGLES, 0, lMods[i].rod.count);
                }
            }
        };

        if (g_splitScreen) {
            glm::mat4 projHalf = glm::perspective(glm::radians(45.f),(float)(SCR_W/2.0f)/SCR_H,0.01f,10.f);
            glViewport(0, 0, SCR_W/2, SCR_H);
            drawScene(g_cam.view(), g_cam.pos(), 0, projHalf);
            glViewport(SCR_W/2, 0, SCR_W/2, SCR_H);
            drawScene(g_topCam.view(), g_topCam.pos(), 1, projHalf);
            glViewport(0, 0, SCR_W, SCR_H);
        } else {
            glViewport(0, 0, SCR_W, SCR_H);
            if (g_camMode == 0) drawScene(g_cam.view(), g_cam.pos(), 0, proj);
            else drawScene(g_topCam.view(), g_topCam.pos(), 1, proj);
        }

        if (glm::distance(g_p.p, lastP.p) > 0.0001f || glm::distance(g_p.r, lastP.r) > 0.01f) {
            printf("[LOG] POSE[XYZ: %.3f %.3f %.3f | RPY: %.1f %.1f %.1f]\n",
                g_p.p.x, g_p.p.y, g_p.p.z, g_p.r.x, g_p.r.y, g_p.r.z);
            printf("      LEGS: [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
                legLengths[0], legLengths[1], legLengths[2], legLengths[3], legLengths[4], legLengths[5]);
            lastP=g_p;
            if(csv.is_open()) {
                csv << currentFrameTime << "," << g_p.p.x << "," << g_p.p.y << "," << g_p.p.z << ","
                    << g_p.r.x << "," << g_p.r.y << "," << g_p.r.z << ","
                    << legLengths[0] << "," << legLengths[1] << "," << legLengths[2] << ","
                    << legLengths[3] << "," << legLengths[4] << "," << legLengths[5] << "\n";
            }
        }
        glfwSwapBuffers(win);
    }
    glfwTerminate(); return 0;
}
