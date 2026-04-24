/*
 * leg_joint_viewer.cpp  (v93 – Reversion/Stability Protocol)
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

#include "DockingSimulation.h"
#include "TripleBuffer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "tinyxml2.h"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <array>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#undef APIENTRY
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#endif

static const int SCR_W = 1920, SCR_H = 1200;  // MUCH LARGER WINDOW
static const int DASHBOARD_W = 380;  // Bigger left panel WITH MORE CONTROLS
static const int EVENTS_W = 0;       // Right panel width (hidden)
static const int VIZ_W = SCR_W - DASHBOARD_W - EVENTS_W;  // Larger central visualization

// Forward declarations
void loadScenario(const std::string& scenarioPath, const std::string& label);
void onKey(GLFWwindow* window, int k, int s, int a, int m);
void renderDashboardUI();
void saveScenarioToXml();
extern std::atomic<bool> g_playScenario;  // Forward declare global
static const int N_LEGS = 6;
static float SCALE = 0.005f;

// ============ DASHBOARD UI STRUCTURES ============
struct DashboardEvent {
    double time;
    std::array<glm::vec3, N_LEGS> forces;  // One force vector per leg
};

struct DashboardState {
    bool showDashboard = true;
    int selectedEventIdx = 0;  // Index of event being edited (starts at 0)
    std::vector<DashboardEvent> events;
    std::string currentScenarioPath;
    bool isDirty = false;
    int selectedScenarioIdx = 0;  // 0=Normal, 1=Off-Center
    bool isPlayingVisualization = false;
};

DashboardState g_dashState;

static const float L_MIN = 0.15f; 
static const float L_MAX = 0.258f; 
static const float CONTACT_VISUAL_LIFT = 0.000f;

static const char* PATHS[N_LEGS] = {
    "assets/models/leg_1.obj", "assets/models/leg_2.obj",
    "assets/models/leg_3.obj", "assets/models/leg_4.obj",
    "assets/models/leg_5.obj", "assets/models/leg_6.obj",
};
static const char* MOD_PLATE = "assets/models/bottomStewart.obj";
static const char* MOD_TARGET = "assets/models/Up_stewartPlatform.obj";
static const char* MOD_RIG = "assets/models/the_base.obj";
static const char* SCENARIO_NORMAL = "scenarios/normal_docking.xml";
static const char* SCENARIO_OFF_CENTER = "scenarios/off_center_contact.xml";

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

Mesh unitQuad() {
    std::vector<float> b = {
        0.f, 0.f, 0.f, 0.f, 0.f, 1.f,
        1.f, 0.f, 0.f, 0.f, 0.f, 1.f,
        1.f, 1.f, 0.f, 0.f, 0.f, 1.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 1.f,
        1.f, 1.f, 0.f, 0.f, 0.f, 1.f,
        0.f, 1.f, 0.f, 0.f, 0.f, 1.f
    };
    return upload(b, 0);
}

// ============ OPENGL DASHBOARD RENDERING ============
void printDashboardText(const std::string& text, int x, int y) {
    printf("[UI @ %d,%d] %s\n", x, y, text.c_str());
}

void renderDashboardUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    // Main Control Panel - Light Theme
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(DASHBOARD_W, SCR_H), ImGuiCond_FirstUseEver);
    
    ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
    
    // Scenario Selection
    ImGui::Text("SCENARIO");
    ImGui::PushItemWidth(-1);
    const char* scenarioItems[] = { "Normal Docking", "Off-Center Contact" };
    if (ImGui::Combo("##scenario", &g_dashState.selectedScenarioIdx, scenarioItems, 2)) {
        g_dashState.isDirty = true;
    }
    ImGui::PopItemWidth();
    
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Spacing();
    
    // Event Selection
    ImGui::Text("EVENT EDITOR");
    if (g_dashState.events.empty()) {
        ImGui::Text("No events loaded. Click LOAD MODEL first.");
    } else {
        ImGui::Text("Total Events: %zu", g_dashState.events.size());
        
        ImGui::Spacing();
        ImGui::Text("Select Event:");
        
        // Event list
        if (ImGui::BeginListBox("##eventselect", ImVec2(-1, 120))) {
            for (size_t i = 0; i < g_dashState.events.size(); ++i) {
                char eventLabel[64];
                sprintf(eventLabel, "Event @ %.1f sec", g_dashState.events[i].time);
                bool isSelected = (g_dashState.selectedEventIdx == (int)i);
                if (ImGui::Selectable(eventLabel, isSelected)) {
                    g_dashState.selectedEventIdx = (int)i;
                }
            }
            ImGui::EndListBox();
        }
        
        ImGui::Spacing();
        ImGui::Spacing();
        
        // Edit selected event
        if (g_dashState.selectedEventIdx >= 0 && g_dashState.selectedEventIdx < (int)g_dashState.events.size()) {
            auto& selectedEvent = g_dashState.events[g_dashState.selectedEventIdx];
            
            ImGui::Text("Editing: Event @ %.1f sec", selectedEvent.time);
            ImGui::Separator();
            ImGui::Spacing();
            
            // Event time input
            if (ImGui::InputDouble("Event Time##edittime", &selectedEvent.time, 0.1, 0.5, "%.1f")) {
                g_dashState.isDirty = true;
            }
            
            ImGui::Spacing();
            ImGui::Text("SENSOR FORCES (N):");
            ImGui::Spacing();
            
            // Sensor force inputs
            for (int i = 0; i < N_LEGS; ++i) {
                char label[32];
                sprintf(label, "Leg %d (X)##fx%d", i + 1, i);
                ImGui::PushItemWidth(-1);
                if (ImGui::InputFloat(label, &selectedEvent.forces[i].x, 0.1f, 0.5f, "%.2f")) {
                    g_dashState.isDirty = true;
                }
                ImGui::PopItemWidth();
                
                sprintf(label, "Leg %d (Y)##fy%d", i + 1, i);
                ImGui::PushItemWidth(-1);
                if (ImGui::InputFloat(label, &selectedEvent.forces[i].y, 0.1f, 0.5f, "%.2f")) {
                    g_dashState.isDirty = true;
                }
                ImGui::PopItemWidth();
                
                sprintf(label, "Leg %d (Z)##fz%d", i + 1, i);
                ImGui::PushItemWidth(-1);
                if (ImGui::InputFloat(label, &selectedEvent.forces[i].z, 0.1f, 0.5f, "%.2f")) {
                    g_dashState.isDirty = true;
                }
                ImGui::PopItemWidth();
                
                ImGui::Spacing();
            }
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        // Add/Delete buttons
        if (ImGui::Button("+ ADD NEW EVENT", ImVec2(-1, 35))) {
            DashboardEvent newEvent;
            newEvent.time = g_dashState.events.empty() ? 1.0 : g_dashState.events.back().time + 1.0;
            for (int i = 0; i < N_LEGS; ++i) {
                newEvent.forces[i] = glm::vec3(0.0f, 0.0f, 0.0f);
            }
            g_dashState.events.push_back(newEvent);
            g_dashState.selectedEventIdx = (int)g_dashState.events.size() - 1;
            g_dashState.isDirty = true;
            printf("[DASHBOARD] New event added at t=%.1f\n", newEvent.time);
        }
        
        ImGui::Spacing();
        
        if (ImGui::Button("- DELETE EVENT", ImVec2(-1, 35))) {
            if (g_dashState.selectedEventIdx >= 0 && g_dashState.selectedEventIdx < (int)g_dashState.events.size()) {
                g_dashState.events.erase(g_dashState.events.begin() + g_dashState.selectedEventIdx);
                if (g_dashState.selectedEventIdx >= (int)g_dashState.events.size() && g_dashState.selectedEventIdx > 0) {
                    g_dashState.selectedEventIdx--;
                }
                g_dashState.isDirty = true;
                printf("[DASHBOARD] Event deleted\n");
            }
        }
    }
    
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Spacing();
    
    // Action Buttons
    if (ImGui::Button("LOAD MODEL", ImVec2(-1, 45))) {
        const char* path = (g_dashState.selectedScenarioIdx == 0) ? SCENARIO_NORMAL : SCENARIO_OFF_CENTER;
        const char* label = (g_dashState.selectedScenarioIdx == 0) ? "Normal Docking" : "Off-Center Contact";
        loadScenario(path, label);
        g_playScenario.store(false);  // Load but don't play yet
        g_dashState.isPlayingVisualization = false;
        printf("[DASHBOARD] Model loaded - ready to visualize\n");
    }
    
    ImGui::Spacing();
    
    if (ImGui::Button("SAVE & RELOAD", ImVec2(-1, 45))) {
        saveScenarioToXml();
        // Reload scenario to use new forces
        if (!g_dashState.currentScenarioPath.empty()) {
            const char* label = (g_dashState.selectedScenarioIdx == 0) ? "Normal Docking" : "Off-Center Contact";
            
            // CRITICAL: Stop playback before reloading
            g_playScenario.store(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait for sim thread to pause
            
            loadScenario(g_dashState.currentScenarioPath, label);
            printf("[DASHBOARD] ✓ Scenario reloaded with new forces!\n");
        }
    }
    
    ImGui::Spacing();
    
    if (ImGui::Button("START VISUALIZATION", ImVec2(-1, 45))) {
        g_playScenario.store(true);  // Start playing
        g_dashState.isPlayingVisualization = true;
        printf("[DASHBOARD] Visualization started\n");
    }
    
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Status: %s | %s", 
        g_dashState.isPlayingVisualization ? "PLAYING" : "READY",
        g_dashState.isDirty ? "MODIFIED" : "SAVED");
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    // Help/Info Section
    if (ImGui::CollapsingHeader("? HELP: What is Contact?")) {
        ImGui::TextWrapped(
            "CONTACT = Docking Event\n\n"
            "Each <Contact> element represents a moment in time when:\n"
            "• The platform's bottom face TOUCHES the upper target\n"
            "• All 6 leg sensors measure compression forces\n"
            "• The forces (fx, fy, fz) are physical measurements\n\n"
            "WORKFLOW:\n"
            "1. LOAD MODEL - Loads all events from XML\n"
            "2. SELECT & EDIT - Modify forces for any event\n"
            "3. SAVE & RELOAD - Saves + reloads scenario\n"
            "4. START VIZ - Plays with YOUR new forces\n\n"
            "KEY: Must SAVE & RELOAD after editing!\n"
            "Otherwise old forces will still be used."
        );
    }
    
    ImGui::End();
    
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void saveScenarioToXml() {
    if (g_dashState.currentScenarioPath.empty()) {
        printf("[ERROR] No scenario path set\n");
        return;
    }
    
    try {
        tinyxml2::XMLDocument doc;
        
        // Load existing XML file
        tinyxml2::XMLError loadResult = doc.LoadFile(g_dashState.currentScenarioPath.c_str());
        if (loadResult != tinyxml2::XML_SUCCESS) {
            printf("[ERROR] Failed to load XML file: %s\n", g_dashState.currentScenarioPath.c_str());
            return;
        }
        
        // Get the Scenario element
        tinyxml2::XMLElement* scenarioElem = doc.FirstChildElement("Scenario");
        if (!scenarioElem) {
            printf("[ERROR] No Scenario element found in XML\n");
            return;
        }
        
        // Get or create Events element
        tinyxml2::XMLElement* eventsElem = scenarioElem->FirstChildElement("Events");
        if (!eventsElem) {
            eventsElem = doc.NewElement("Events");
            scenarioElem->InsertEndChild(eventsElem);
        }
        
        // Remove all existing Contact elements
        tinyxml2::XMLElement* contactElem = eventsElem->FirstChildElement("Contact");
        while (contactElem) {
            tinyxml2::XMLElement* nextElem = contactElem->NextSiblingElement("Contact");
            eventsElem->DeleteChild(contactElem);
            contactElem = nextElem;
        }
        
        // Add new Contact elements from dashboard events
        for (size_t i = 0; i < g_dashState.events.size(); ++i) {
            const auto& evt = g_dashState.events[i];
            
            tinyxml2::XMLElement* newContact = doc.NewElement("Contact");
            newContact->SetAttribute("time", evt.time);
            
            // Add Force elements for each leg
            for (int j = 0; j < N_LEGS; ++j) {
                tinyxml2::XMLElement* forceElem = doc.NewElement("Force");
                forceElem->SetAttribute("fx", evt.forces[j].x);
                forceElem->SetAttribute("fy", evt.forces[j].y);
                forceElem->SetAttribute("fz", evt.forces[j].z);
                forceElem->SetAttribute("sensor", j + 1);
                newContact->InsertEndChild(forceElem);
            }
            
            eventsElem->InsertEndChild(newContact);
            printf("[SAVE] Event %zu - Time: %.3f\n", i, evt.time);
        }
        
        // Save file
        tinyxml2::XMLError saveResult = doc.SaveFile(g_dashState.currentScenarioPath.c_str());
        if (saveResult == tinyxml2::XML_SUCCESS) {
            g_dashState.isDirty = false;
            printf("[SAVE] ✓ Scenario saved to: %s\n", g_dashState.currentScenarioPath.c_str());
            printf("[SAVE] Total events saved: %zu\n", g_dashState.events.size());
        } else {
            printf("[ERROR] Failed to save XML file\n");
        }
    } catch (const std::exception& e) {
        printf("[ERROR] Exception during save: %s\n", e.what());
    }
}

struct Cam {
    float yaw=45, pitch=25, dist=0.6f; glm::vec3 tgt={0,0.1,0};
    glm::vec3 pos()const{ float y=glm::radians(yaw),p=glm::radians(pitch); return tgt+dist*glm::vec3(cosf(p)*sinf(y),sinf(p),cosf(p)*cosf(y)); }
    glm::mat4 view()const{ return glm::lookAt(pos(),tgt,{0,1,0}); }
} g_cam;

struct Pose { glm::vec3 p; glm::vec3 r; } g_p, g_def = {{0,0.025,0}, {0,0,0}}, g_test;

struct SimulationSnapshot {
    Pose combinedPose = {};
    Pose basePose = {};
    Pose suspendedPose = {};
    double simTime = 0.0;
    DockingFeasibilityState dockingState = DockingFeasibilityState::APPROACHING;
    bool contactActive = false;
    float contactLateral = 0.0f;
    float contactAxialGap = 0.0f;
    bool valid = false;
};



std::unique_ptr<DockingSimulation> g_dockingSim;
TripleBuffer<SimulationSnapshot> g_snapshotBuffer;
SimulationSnapshot g_latestSnapshot;
std::mutex g_simMutex;
std::condition_variable g_simCv;
std::thread g_simThread;
std::atomic<bool> g_playScenario{false};
std::atomic<bool> g_exitSimulationThread{false};
std::string g_loadedScenarioLabel = "Manual";
Pose g_basePoseDisplay = {};
Pose g_suspendedPoseDisplay = {};
std::vector<glm::vec3> g_pBase, g_pTopHome;
std::ofstream g_stepCsv;
std::chrono::steady_clock::time_point g_csvStartTime;
std::chrono::steady_clock::time_point g_lastPhysicsStepTime;
bool g_hasLastPhysicsStepTime = false;
long long g_jitterSampleCount = 0;
long long g_jitterMinUs = 0;
long long g_jitterMaxUs = 0;
long long g_jitterAccumUs = 0;
bool g_csvHeaderWritten = false;
float g_contactVisualBlend = 0.0f;
float g_contactRecoilOffset = 0.0f;
float g_contactRecoilVelocity = 0.0f;
DockingFeasibilityState g_lastRecoilState = DockingFeasibilityState::APPROACHING;
std::chrono::steady_clock::time_point g_lastVisualFrameTime;
bool g_hasLastVisualFrameTime = false;
static double mx, my; static bool md=false;

const char* contactAnsiColor(DockingFeasibilityState state) {
    if (state == DockingFeasibilityState::VALID_CONTACT) return "\x1b[32m";
    if (state == DockingFeasibilityState::APPROACHING) return "\x1b[31m";
    return "\x1b[33m";
}



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

Pose lerpPose(const Pose& from, const Pose& to, float alpha) {
    Pose out = from;
    out.p = glm::mix(from.p, to.p, alpha);
    auto lerpAngle = [alpha](float a, float b) {
        float delta = b - a;
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        return a + delta * alpha;
    };
    out.r.x = lerpAngle(from.r.x, to.r.x);
    out.r.y = lerpAngle(from.r.y, to.r.y);
    out.r.z = lerpAngle(from.r.z, to.r.z);
    return out;
}

SimulationSnapshot buildSnapshotFromSimulation(const DockingSimulation& simulation) {
    SimulationSnapshot snapshot;
    snapshot.combinedPose = toViewerPose(simulation.getCombinedPose());
    snapshot.basePose = toViewerPose(simulation.getBaseTargetPose());
    snapshot.suspendedPose = toViewerPose(simulation.getSuspendedTargetPose());
    snapshot.simTime = simulation.getCurrentTime();
    snapshot.dockingState = simulation.getDockingState();
    const Pose upperPose = {{0.0f, 0.0343f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    snapshot.contactLateral = std::sqrt(
        (snapshot.combinedPose.p.x - upperPose.p.x) * (snapshot.combinedPose.p.x - upperPose.p.x) +
        (snapshot.combinedPose.p.z - upperPose.p.z) * (snapshot.combinedPose.p.z - upperPose.p.z));
    snapshot.contactAxialGap = upperPose.p.y - snapshot.combinedPose.p.y;
    snapshot.contactActive = snapshot.dockingState == DockingFeasibilityState::VALID_CONTACT;
    snapshot.valid = true;
    return snapshot;
}

void publishSimulationSnapshotLocked() {
    if (!g_dockingSim) return;
    g_snapshotBuffer.write(buildSnapshotFromSimulation(*g_dockingSim));
}

void syncViewerPoseFromSnapshot() {
    g_latestSnapshot = g_snapshotBuffer.read();
}

void snapViewerPoseToSnapshot() {
    if (!g_latestSnapshot.valid) return;
    g_p = g_latestSnapshot.combinedPose;
    g_basePoseDisplay = g_latestSnapshot.basePose;
    g_suspendedPoseDisplay = g_latestSnapshot.suspendedPose;
}

void smoothViewerPoseToSnapshot(float alpha) {
    if (!g_latestSnapshot.valid) return;
    g_p = lerpPose(g_p, g_latestSnapshot.combinedPose, alpha);
    g_basePoseDisplay = lerpPose(g_basePoseDisplay, g_latestSnapshot.basePose, alpha);
    g_suspendedPoseDisplay = lerpPose(g_suspendedPoseDisplay, g_latestSnapshot.suspendedPose, alpha);
}

std::array<float, N_LEGS> computeLegLengthsForPose(const Pose& pose) {
    std::array<float, N_LEGS> lengths{};
    glm::mat4 T = glm::translate(glm::mat4(1.f), pose.p) * glm::toMat4(glm::quat(glm::radians(pose.r)));
    for (int i = 0; i < N_LEGS; ++i) {
        const glm::vec3 top = glm::vec3(T * glm::vec4(g_pTopHome[i], 1.f));
        lengths[i] = glm::length(top - g_pBase[i]);
    }
    return lengths;
}

void openStepCsvLocked() {
    if (g_stepCsv.is_open()) g_stepCsv.close();
    g_stepCsv.open("leg_joint_viewer_step_log.csv", std::ios::trunc);
    g_stepCsv << "wall_time_us,delta_us,target_step_us,sim_time_s,"
                 "jitter_us,jitter_min_us,jitter_avg_us,jitter_max_us,"
                 "combined_x,combined_y,combined_z,combined_roll_deg,combined_pitch_deg,combined_yaw_deg,"
                 "base_x,base_y,base_z,base_roll_deg,base_pitch_deg,base_yaw_deg,"
                 "suspended_x,suspended_y,suspended_z,suspended_roll_deg,suspended_pitch_deg,suspended_yaw_deg,"
                 "docking_state,contact_active,contact_lateral_m,contact_axial_gap_m,"
                 "leg_1,leg_2,leg_3,leg_4,leg_5,leg_6\n";
    printf("[CSV] Fresh step log: leg_joint_viewer_step_log.csv\n");
    g_csvStartTime = std::chrono::steady_clock::now();
    g_lastPhysicsStepTime = {};
    g_hasLastPhysicsStepTime = false;
    g_jitterSampleCount = 0;
    g_jitterMinUs = 0;
    g_jitterMaxUs = 0;
    g_jitterAccumUs = 0;
    g_csvHeaderWritten = true;
    g_contactVisualBlend = 0.0f;
    g_lastVisualFrameTime = {};
    g_hasLastVisualFrameTime = false;
}

void logPhysicsStepCsvLocked(double stepSize) {
    if (!g_stepCsv.is_open() || !g_dockingSim) return;

    const SimulationSnapshot snapshot = buildSnapshotFromSimulation(*g_dockingSim);
    auto legLengths = computeLegLengthsForPose(snapshot.combinedPose);
    if (snapshot.contactActive) {
        for (auto& legLength : legLengths) {
            legLength = L_MAX;
        }
    }
    const auto now = std::chrono::steady_clock::now();
    const auto wallTimeUs =
        std::chrono::duration_cast<std::chrono::microseconds>(now - g_csvStartTime).count();
    const auto deltaUs =
        g_hasLastPhysicsStepTime
            ? std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastPhysicsStepTime).count()
            : 0LL;
    const long long targetStepUs = static_cast<long long>(stepSize * 1000000.0);
    const long long jitterUs = g_hasLastPhysicsStepTime ? (deltaUs - targetStepUs) : 0LL;

    if (g_hasLastPhysicsStepTime) {
        if (g_jitterSampleCount == 0) {
            g_jitterMinUs = jitterUs;
            g_jitterMaxUs = jitterUs;
        } else {
            g_jitterMinUs = std::min(g_jitterMinUs, jitterUs);
            g_jitterMaxUs = std::max(g_jitterMaxUs, jitterUs);
        }
        g_jitterAccumUs += jitterUs;
        g_jitterSampleCount++;
    }

    const double jitterAvgUs =
        g_jitterSampleCount > 0
            ? static_cast<double>(g_jitterAccumUs) / static_cast<double>(g_jitterSampleCount)
            : 0.0;

    g_stepCsv << wallTimeUs << ','
              << deltaUs << ','
              << targetStepUs << ','
              << std::fixed << std::setprecision(6)
              << snapshot.simTime << ','
              << jitterUs << ','
              << g_jitterMinUs << ','
              << jitterAvgUs << ','
              << g_jitterMaxUs << ','
              << snapshot.combinedPose.p.x << ',' << snapshot.combinedPose.p.y << ',' << snapshot.combinedPose.p.z << ','
              << snapshot.combinedPose.r.x << ',' << snapshot.combinedPose.r.y << ',' << snapshot.combinedPose.r.z << ','
              << snapshot.basePose.p.x << ',' << snapshot.basePose.p.y << ',' << snapshot.basePose.p.z << ','
              << snapshot.basePose.r.x << ',' << snapshot.basePose.r.y << ',' << snapshot.basePose.r.z << ','
              << snapshot.suspendedPose.p.x << ',' << snapshot.suspendedPose.p.y << ',' << snapshot.suspendedPose.p.z << ','
              << snapshot.suspendedPose.r.x << ',' << snapshot.suspendedPose.r.y << ',' << snapshot.suspendedPose.r.z << ','
              << static_cast<int>(snapshot.dockingState) << ','
              << (snapshot.contactActive ? 1 : 0) << ','
              << snapshot.contactLateral << ','
              << snapshot.contactAxialGap << ','
              << legLengths[0] << ',' << legLengths[1] << ',' << legLengths[2] << ','
              << legLengths[3] << ',' << legLengths[4] << ',' << legLengths[5] << '\n';
    g_stepCsv.flush();

    g_lastPhysicsStepTime = now;
    g_hasLastPhysicsStepTime = true;
}

void simulationThreadMain() {
    using Clock = std::chrono::steady_clock;
#ifdef _WIN32
    timeBeginPeriod(1);
    HANDLE process = GetCurrentProcess();
    SetPriorityClass(process, HIGH_PRIORITY_CLASS);
    HANDLE threadHandle = GetCurrentThread();
    SetThreadPriority(threadHandle, THREAD_PRIORITY_TIME_CRITICAL);
#endif
    auto nextTick = Clock::now();

    while (!g_exitSimulationThread.load()) {
        double stepSize = 0.002;
        bool shouldStep = false;

        {
            std::unique_lock<std::mutex> lock(g_simMutex);
            g_simCv.wait_for(lock, std::chrono::milliseconds(10), [] {
                return g_exitSimulationThread.load() || (g_playScenario.load() && g_dockingSim);
            });

            if (g_exitSimulationThread.load()) break;
            if (!g_playScenario.load() || !g_dockingSim) {
                nextTick = Clock::now();
                continue;
            }

            stepSize = g_dockingSim->getScenario()->config.timestep;
            g_dockingSim->step();
            publishSimulationSnapshotLocked();
            logPhysicsStepCsvLocked(stepSize);
            shouldStep = true;
        }

        if (!shouldStep) continue;

        const auto interval = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(stepSize));
        const auto now = Clock::now();
        if (nextTick < now) nextTick = now;
        nextTick += interval;

        const auto spinThreshold = std::chrono::microseconds(300);
        while (!g_exitSimulationThread.load()) {
            const auto current = Clock::now();
            if (current >= nextTick) break;
            const auto remaining = nextTick - current;
            if (remaining > spinThreshold) {
                std::this_thread::sleep_for(std::chrono::microseconds(remaining > std::chrono::microseconds(1000)
                    ? std::chrono::duration_cast<std::chrono::microseconds>(remaining - spinThreshold)
                    : std::chrono::microseconds(50)));
            } else {
                std::atomic_thread_fence(std::memory_order_seq_cst);
            }
        }
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

void loadScenario(const std::string& scenarioPath, const std::string& label) {
    const std::string resolvedPath = findRuntimePath(scenarioPath);

    try {
        auto nextSimulation = std::make_unique<DockingSimulation>(resolvedPath);
        {
            std::lock_guard<std::mutex> lock(g_simMutex);
            g_dockingSim = std::move(nextSimulation);
            openStepCsvLocked();
            publishSimulationSnapshotLocked();
        }
        g_playScenario.store(false);
        g_loadedScenarioLabel = label;
        g_lastVisualFrameTime = {};
        g_hasLastVisualFrameTime = false;
        syncViewerPoseFromSnapshot();
        snapViewerPoseToSnapshot();
        
        // Set dashboard scenario path
        g_dashState.currentScenarioPath = resolvedPath;
        g_dashState.isDirty = false;
        
        // Load events from XML
        g_dashState.events.clear();
        try {
            tinyxml2::XMLDocument doc;
            if (doc.LoadFile(resolvedPath.c_str()) == tinyxml2::XML_SUCCESS) {
                tinyxml2::XMLElement* scenarioElem = doc.FirstChildElement("Scenario");
                if (scenarioElem) {
                    tinyxml2::XMLElement* eventsElem = scenarioElem->FirstChildElement("Events");
                    if (eventsElem) {
                        tinyxml2::XMLElement* contactElem = eventsElem->FirstChildElement("Contact");
                        while (contactElem) {
                            DashboardEvent event;
                            double timeValue = 0.0;
                            contactElem->QueryDoubleAttribute("time", &timeValue);
                            event.time = timeValue;
                            
                            // Load forces for each leg
                            tinyxml2::XMLElement* forceElem = contactElem->FirstChildElement("Force");
                            int sensorIdx = 0;
                            while (forceElem && sensorIdx < N_LEGS) {
                                double fx = 0.0, fy = 0.0, fz = 0.0;
                                forceElem->QueryDoubleAttribute("fx", &fx);
                                forceElem->QueryDoubleAttribute("fy", &fy);
                                forceElem->QueryDoubleAttribute("fz", &fz);
                                event.forces[sensorIdx] = glm::vec3(fx, fy, fz);
                                sensorIdx++;
                                forceElem = forceElem->NextSiblingElement("Force");
                            }
                            
                            g_dashState.events.push_back(event);
                            contactElem = contactElem->NextSiblingElement("Contact");
                        }
                    }
                }
            }
            if (!g_dashState.events.empty()) {
                g_dashState.selectedEventIdx = 0;
                printf("[DASHBOARD] Loaded %zu events from XML\n", g_dashState.events.size());
            }
        } catch (const std::exception& e) {
            printf("[ERROR] Failed to load events: %s\n", e.what());
        }
        
        printf("[SCENARIO] Loaded %s from %s\n", label.c_str(), resolvedPath.c_str());
        printf("[SCENARIO] Physics thread ready. Press P to run at the scenario timestep on the worker thread.\n");
        printf("[SCENARIO] Step log CSV: leg_joint_viewer_step_log.csv\n");
        printf("[DASHBOARD] Ready to edit scenario. Press D to show dashboard.\n");
    } catch (const std::exception& e) {
        {
            std::lock_guard<std::mutex> lock(g_simMutex);
            g_dockingSim.reset();
            if (g_stepCsv.is_open()) g_stepCsv.close();
        }
        g_playScenario.store(false);
        printf("[SCENARIO] Failed to load %s: %s\n", label.c_str(), e.what());
    }
}

void onBtn(GLFWwindow*,int b,int a,int){ if(b==GLFW_MOUSE_BUTTON_LEFT)md=(a==GLFW_PRESS); }
void onPos(GLFWwindow*,double dx,double dy){ if(md){g_cam.yaw-=(float)(dx-mx)*.4f;g_cam.pitch+=(float)(dy-my)*.4f;g_cam.pitch=glm::clamp(g_cam.pitch,-89.f,89.f);} mx=dx;my=dy; }
void onScr(GLFWwindow*,double,double d){ 
    // ONLY zoom camera if NOT scrolling over ImGui
    if (!ImGui::GetIO().WantCaptureMouse) {
        g_cam.dist=glm::clamp(g_cam.dist-(float)d*.05f,0.05f,5.f);
    }
}

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

    // ========== DASHBOARD CONTROLS ==========
    if(k==GLFW_KEY_D) {
        g_dashState.showDashboard = !g_dashState.showDashboard;
        printf("[DASHBOARD] Dashboard %s\n", g_dashState.showDashboard ? "ON" : "OFF");
        return;
    }

    if(k==GLFW_KEY_1) { loadScenario(SCENARIO_NORMAL, "Normal Docking"); return; }
    if(k==GLFW_KEY_5) { loadScenario(SCENARIO_OFF_CENTER, "Off-Center Contact"); return; }
    if(k==GLFW_KEY_P) {
        {
            std::lock_guard<std::mutex> lock(g_simMutex);
            if(!g_dockingSim) { printf("[SCENARIO] Load a scenario first with 1-5.\n"); return; }
        }
        const bool enablePlayback = !g_playScenario.load();
        g_playScenario.store(enablePlayback);
        if (enablePlayback) g_simCv.notify_one();
        printf("[SCENARIO] Playback %s on worker thread (%s)\n",
            enablePlayback ? "ON" : "OFF",
            g_loadedScenarioLabel.c_str());
        return;
    }
    g_test = g_p;
    if(k==GLFW_KEY_A || k==GLFW_KEY_LEFT) g_test.p.x-=s;
    if(k==GLFW_KEY_D || k==GLFW_KEY_RIGHT) g_test.p.x+=s;
    if(k==GLFW_KEY_UP) g_test.p.y+=s;
    if(k==GLFW_KEY_DOWN) g_test.p.y-=s;
    if(k==GLFW_KEY_PAGE_UP) g_test.p.z+=s;
    if(k==GLFW_KEY_PAGE_DOWN) g_test.p.z-=s;
    if(k==GLFW_KEY_W) g_test.r.x+=r; if(k==GLFW_KEY_S) g_test.r.x-=r;
    if(k==GLFW_KEY_Q) g_test.r.y+=r; if(k==GLFW_KEY_E) g_test.r.y-=r;
    if(k==GLFW_KEY_I) g_test.r.z+=r; if(k==GLFW_KEY_U) g_test.r.z-=r;
    if(k==GLFW_KEY_R) {
        g_p=g_def;
        g_basePoseDisplay = {};
        g_suspendedPoseDisplay = {};
        g_playScenario.store(false);
        g_contactRecoilOffset = 0.0f;
        g_contactRecoilVelocity = 0.0f;
        g_lastRecoilState = DockingFeasibilityState::APPROACHING;
        g_lastVisualFrameTime = {};
        g_hasLastVisualFrameTime = false;
        {
            std::lock_guard<std::mutex> lock(g_simMutex);
            if (g_dockingSim) publishSimulationSnapshotLocked();
        }
        syncViewerPoseFromSnapshot();
        snapViewerPoseToSnapshot();
        printf("[LOG] RESET POSE\n");
        return;
    }
    if(g_playScenario.load()) return;
    if(validate(g_test)) { g_p = g_test; }
    else { printf("[!] MECHANICAL STOP: Actuator Collision Avoided.\n"); }
}

int main(){
    if(!glfwInit())return -1;
    glfwWindowHint(GLFW_SAMPLES,4);
    GLFWwindow* win=glfwCreateWindow(SCR_W,SCR_H,"Stewart Engineering Rig Viewer v93 - DOCKING FACE ENABLED",0,0);
    glfwMakeContextCurrent(win); glfwSetMouseButtonCallback(win,onBtn); glfwSetCursorPosCallback(win,onPos); glfwSetScrollCallback(win,onScr); glfwSetKeyCallback(win,onKey);
    gladLoadGL(glfwGetProcAddress);
    
    // ImGui Init - Light Theme with LARGER text
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsLight();
    ImGui::GetStyle().ScaleAllSizes(2.2f);  // HUGE UI elements
    ImGuiIO& io = ImGui::GetIO();
    // Try loading Arial, fallback to default if not found
    const char* fontPaths[] = {
        "C:/Windows/Fonts/arial.ttf",
        "C:/WINDOWS/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
    };
    bool fontLoaded = false;
    for (const char* fontPath : fontPaths) {
        if (std::filesystem::exists(fontPath)) {
            io.Fonts->AddFontFromFileTTF(fontPath, 28.0f);  // GIANT font
            fontLoaded = true;
            break;
        }
    }
    if (!fontLoaded) {
        io.Fonts->AddFontDefault();  // Use default ImGui font
    }
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    io.IniFilename = nullptr;  // Disable ini file storage


    glEnable(GL_DEPTH_TEST);
    auto c=[](GLenum t,const char* s){GLuint h=glCreateShader(t);glShaderSource(h,1,&s,0);glCompileShader(h);return h;};
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
    float d=max(dot(N,L),0.0)*0.7; fC=vec4(uCol*(0.65+d),1.0);
})"));  // Increased ambient lighting and diffuse
    glLinkProgram(pr);
    GLuint locClip = glGetUniformLocation(pr,"uClipPlane");
    GLuint locM = glGetUniformLocation(pr,"uM");

    Model platMod = loadSimple(MOD_PLATE);
    Model targetMod = loadSimple(MOD_TARGET);
    Model rigMod = loadSimple(MOD_RIG);
    Model lMods[N_LEGS]; for(int i=0;i<N_LEGS;++i) lMods[i]=loadLeg(PATHS[i]);
    Mesh dashboardQuad = unitQuad();
    glm::vec3 pltC = (platMod.bMin + platMod.bMax) * 0.5f;
    glm::mat4 v16WorldT = glm::scale(glm::mat4(1.f),glm::vec3(SCALE)) * glm::translate(glm::mat4(1.f),-pltC);

    float minY=1e9f;
    for(int i=0;i<N_LEGS;++i){
        g_pBase.push_back(glm::vec3(v16WorldT*glm::vec4(lMods[i].jb_local,1.f)));
        g_pTopHome.push_back(glm::vec3(v16WorldT*glm::vec4(lMods[i].jt_local,1.f)));
        minY = std::min(minY, g_pBase[i].y);
    }
    {
        std::lock_guard<std::mutex> lock(g_simMutex);
        openStepCsvLocked();
    }
    g_p = g_def; Pose lastP;
    g_simThread = std::thread(simulationThreadMain);
    loadScenario(SCENARIO_NORMAL, "Normal Docking");

    printf("\n========= STEWART PLATFORM VIEWER =========\n");
    printf("[1] Normal docking\n");
    printf("[5] Off-center docking\n");
    printf("[P] Play/Pause worker-thread physics playback\n");
    printf("[R] Reset pose / stop playback\n");
    printf("\n==== DASHBOARD (Press D to toggle) ====\n");
    printf("[D]   Toggle Dashboard UI\n");
    printf("    - Use slider to adjust leg forces\n");
    printf("    - Click +/- buttons to manage events\n");
    printf("    - Select events from list\n");
    printf("    - Click 'Save to XML' to persist changes\n");
    printf("=========================================\n\n");

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents();
        if (g_playScenario.load()) {
            syncViewerPoseFromSnapshot();
        }
        glClearColor(0.95f, 0.95f, 0.95f, 1);  // Bright background
        glDisable(GL_SCISSOR_TEST);  // Disable scissor to clear everything
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);   // Re-enable for visualization area
        
        // Adjust viewport for dashboard panels
        int vizX = g_dashState.showDashboard ? DASHBOARD_W : 0;
        int vizW = g_dashState.showDashboard ? VIZ_W : SCR_W;
        int vizH = SCR_H;
        
        const glm::mat4 projScene = glm::perspective(glm::radians(45.f), static_cast<float>(vizW) / vizH, 0.01f, 10.f);
        
        glViewport(vizX, 0, vizW, vizH);
        glScissor(vizX, 0, vizW, vizH);
        glm::mat4 VP=projScene*g_cam.view(); glUseProgram(pr);
        glUniform3fv(glGetUniformLocation(pr,"uCam"),1,glm::value_ptr(g_cam.pos()));
        glUniform4f(locClip, 0, 0, 0, 10000.f); 

        const auto nowFrame = std::chrono::steady_clock::now();
        float visualDt = 1.0f / 60.0f;
        if (g_hasLastVisualFrameTime) {
            visualDt = std::chrono::duration<float>(nowFrame - g_lastVisualFrameTime).count();
            visualDt = std::clamp(visualDt, 1.0f / 240.0f, 1.0f / 24.0f);
        }
        g_lastVisualFrameTime = nowFrame;
        g_hasLastVisualFrameTime = true;

        const float targetBlend = (g_latestSnapshot.valid && g_latestSnapshot.contactActive) ? 1.0f : 0.0f;
        g_contactVisualBlend = g_contactVisualBlend * 0.85f + targetBlend * 0.15f;

        if (g_playScenario.load()) {
            const float poseAlpha = std::clamp(visualDt * 6.5f, 0.0f, 0.18f);
            smoothViewerPoseToSnapshot(poseAlpha);
        }

        if (g_latestSnapshot.valid && g_latestSnapshot.dockingState != g_lastRecoilState) {
            if (g_latestSnapshot.dockingState == DockingFeasibilityState::VALID_CONTACT) {
                const float compression = std::clamp(-g_latestSnapshot.contactAxialGap, 0.0f, 0.060f);
                g_contactRecoilVelocity = -0.0015f - compression * 0.12f;
                printf("[RECOIL] kickback armed\n");
            } else if (g_latestSnapshot.dockingState == DockingFeasibilityState::INVALID_CONTACT) {
                g_contactRecoilVelocity = -0.0010f;
                printf("[RECOIL] invalid-contact bounce armed\n");
            }
            g_lastRecoilState = g_latestSnapshot.dockingState;
        }

        const float compression = (g_latestSnapshot.valid && g_latestSnapshot.contactActive)
            ? std::clamp(-g_latestSnapshot.contactAxialGap, 0.0f, 0.040f)
            : 0.0f;

        const float recoilMass = 1.0f;
        const float recoilSpring = g_latestSnapshot.valid && g_latestSnapshot.contactActive ? 8.0f : 3.0f;
        const float recoilDamping = g_latestSnapshot.valid && g_latestSnapshot.contactActive ? 2.6f : 1.7f;
        const float targetOffset = g_latestSnapshot.valid && g_latestSnapshot.contactActive
            ? std::clamp(-compression * 0.85f, -0.040f, 0.0f)
            : 0.0f;
        const float recoilAccel = (recoilSpring * (targetOffset - g_contactRecoilOffset)
                                   - recoilDamping * g_contactRecoilVelocity) / recoilMass;
        g_contactRecoilVelocity += recoilAccel * visualDt;
        g_contactRecoilOffset += g_contactRecoilVelocity * visualDt;
        g_contactRecoilVelocity = std::clamp(g_contactRecoilVelocity, -0.012f, 0.008f);
        g_contactRecoilOffset = std::clamp(g_contactRecoilOffset, -0.050f, 0.010f);

        if (!(g_latestSnapshot.valid && g_latestSnapshot.contactActive) &&
            std::abs(g_contactRecoilOffset) < 0.00025f &&
            std::abs(g_contactRecoilVelocity) < 0.00025f) {
            g_contactRecoilOffset = 0.0f;
            g_contactRecoilVelocity = 0.0f;
        }

        Pose visualLowerPose = g_p;
        visualLowerPose.p.y += CONTACT_VISUAL_LIFT * g_contactVisualBlend;
        visualLowerPose.p.y += g_contactRecoilOffset;
        glm::mat4 ctrlT = glm::translate(glm::mat4(1.f), visualLowerPose.p) * glm::toMat4(glm::quat(glm::radians(visualLowerPose.r)));
        
        // Render RIG Structure (Machined Grey)
        glUniform4f(locClip, 0, 0, 0, 10000.f);
        glm::mat4 rigM = v16WorldT;
        glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*rigM));
        glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(rigM));
        glUniform3f(glGetUniformLocation(pr,"uCol"), 0.50, 0.52, 0.55); 
        glBindVertexArray(rigMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, rigMod.casing.count);

        // Render Plate (Silver)
        glUniform4f(locClip, 0, 1, 0, -minY - 0.05f);
        glm::mat4 pM = glm::translate(glm::mat4(1.f), visualLowerPose.p) * glm::toMat4(glm::quat(glm::radians(visualLowerPose.r))) * v16WorldT;
        glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*pM));
        glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(pM));
        glUniform3f(glGetUniformLocation(pr,"uCol"), 0.72, 0.74, 0.76); 
        glBindVertexArray(platMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, platMod.casing.count);

        // Render Static Upper Target (Upright at Y=0.0343)
        glUniform4f(locClip, 0, 0, 0, 10000.f);
        glm::mat4 targetT = glm::translate(glm::mat4(1.f), {0, 0.0343f, 0}) * v16WorldT;
        glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*targetT));
        glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(targetT));
        glUniform3f(glGetUniformLocation(pr,"uCol"), 0.72, 0.74, 0.76); 
        glBindVertexArray(targetMod.casing.vao); glDrawArrays(GL_TRIANGLES, 0, targetMod.casing.count);
        float legLengths[N_LEGS];
        for(int i=0; i<N_LEGS; ++i) {
            glm::vec3 B = g_pBase[i], T = glm::vec3(ctrlT * glm::vec4(g_pTopHome[i], 1.f)), vec = T-B;
            float curL = glm::length(vec); legLengths[i] = curL;
            glm::quat rotQ = glm::rotation(glm::normalize(g_pTopHome[i]-g_pBase[i]), glm::normalize(vec));
            glm::mat4 rotM = glm::toMat4(rotQ);

            // Casing (Machined Grey)
            glm::mat4 matC = glm::translate(glm::mat4(1.f), B) * rotM * glm::translate(glm::mat4(1.f), -B) * v16WorldT;
            glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*matC)); 
            glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(matC));
            glUniform3f(glGetUniformLocation(pr,"uCol"), 0.58, 0.60, 0.62); 
            glBindVertexArray(lMods[i].casing.vao); glDrawArrays(GL_TRIANGLES, 0, lMods[i].casing.count);

            // Rod (Chrome)
            if(lMods[i].hasTwo) {
                glm::mat4 matR = glm::translate(glm::mat4(1.f), T) * rotM * glm::translate(glm::mat4(1.f), -g_pTopHome[i]) * v16WorldT;
                glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*matR)); 
                glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(matR));
                glUniform3f(glGetUniformLocation(pr,"uCol"), 0.88, 0.88, 0.92); 
                glBindVertexArray(lMods[i].rod.vao); glDrawArrays(GL_TRIANGLES, 0, lMods[i].rod.count);
            }
            auto dSph=[&](Mesh& m, glm::vec3 p, glm::vec3 c){
                glm::mat4 mT=glm::translate(glm::mat4(1.f),p);
                glUniformMatrix4fv(glGetUniformLocation(pr,"uMVP"),1,0,glm::value_ptr(VP*mT));
                glUniformMatrix4fv(locM, 1, 0, glm::value_ptr(mT));
                glUniform3fv(glGetUniformLocation(pr,"uCol"),1,glm::value_ptr(c));
                glBindVertexArray(m.vao); glDrawArrays(GL_TRIANGLES, 0, m.count);
            };
            // dSph(sB,B,{0.9,0.8,0.1}); dSph(sT,T,{0.9,0.1,0.2});
        }

        if (g_latestSnapshot.valid && g_latestSnapshot.contactActive) {
            for (float& legLength : legLengths) {
                legLength = L_MAX;
            }
        }

        static DockingFeasibilityState lastLoggedState = DockingFeasibilityState::APPROACHING;
        bool stateChanged = false;
        if (g_latestSnapshot.valid && g_latestSnapshot.dockingState != lastLoggedState) {
            stateChanged = true;
            lastLoggedState = g_latestSnapshot.dockingState;
        }

        if (stateChanged || glm::distance(g_p.p, lastP.p) > 0.0001f || glm::distance(g_p.r, lastP.r) > 0.01f) {
            if (stateChanged) {
                const char* stateStr = "APPROACHING";
                if (lastLoggedState == DockingFeasibilityState::VALID_CONTACT) stateStr = "VALID CONTACT";
                if (lastLoggedState == DockingFeasibilityState::INVALID_CONTACT) stateStr = "BAD DOCKING";
                printf("[STATUS] %s\n", stateStr);
            }
            if (g_latestSnapshot.valid) {
                printf("%s[CONTACT] active=%d state=%d lateral=%.3f axial_gap=%.3f lift=%.3f\x1b[0m\n",
                    contactAnsiColor(g_latestSnapshot.dockingState),
                    g_latestSnapshot.contactActive ? 1 : 0,
                    static_cast<int>(g_latestSnapshot.dockingState),
                    g_latestSnapshot.contactLateral,
                    g_latestSnapshot.contactAxialGap,
                    CONTACT_VISUAL_LIFT * g_contactVisualBlend);
            }
            printf("[LOG] COMBINED[XYZ: %.3f %.3f %.3f | RPY: %.1f %.1f %.1f]\n",
                g_p.p.x, g_p.p.y, g_p.p.z, g_p.r.x, g_p.r.y, g_p.r.z);
            if (g_latestSnapshot.valid) {
                printf("      BASE[XYZ: %.3f %.3f %.3f | RPY: %.1f %.1f %.1f]\n",
                    g_basePoseDisplay.p.x, g_basePoseDisplay.p.y, g_basePoseDisplay.p.z,
                    g_basePoseDisplay.r.x, g_basePoseDisplay.r.y, g_basePoseDisplay.r.z);
                printf("      SUSP[XYZ: %.3f %.3f %.3f | RPY: %.1f %.1f %.1f]\n",
                    g_suspendedPoseDisplay.p.x, g_suspendedPoseDisplay.p.y, g_suspendedPoseDisplay.p.z,
                    g_suspendedPoseDisplay.r.x, g_suspendedPoseDisplay.r.y, g_suspendedPoseDisplay.r.z);
            }
            printf("      LEGS: [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
                legLengths[0], legLengths[1], legLengths[2], legLengths[3], legLengths[4], legLengths[5]);
            lastP=g_p;
        }
        
        // Render ImGui Dashboard UI (disable scissor test for ImGui rendering)
        glDisable(GL_SCISSOR_TEST);
        glViewport(0, 0, SCR_W, SCR_H);  // Reset viewport for ImGui
        if (g_dashState.showDashboard) {
            renderDashboardUI();
        }
        
        glfwSwapBuffers(win);
    }
    g_exitSimulationThread.store(true);
    g_simCv.notify_one();
    if (g_simThread.joinable()) g_simThread.join();
    if (g_stepCsv.is_open()) g_stepCsv.close();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate(); return 0;
}
