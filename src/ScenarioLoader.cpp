#include "ScenarioLoader.h"
#include <tinyxml2.h>
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace tinyxml2;

std::shared_ptr<Scenario> ScenarioLoader::loadFromFile(const std::string& filepath) {
    XMLDocument doc;
    XMLError error = doc.LoadFile(filepath.c_str());
    
    if (error != XML_SUCCESS) {
        std::stringstream ss;
        ss << "Failed to load XML file: " << filepath
           << " (Error code: " << static_cast<int>(error) << ")";
        throw std::runtime_error(ss.str());
    }
    
    XMLElement* rootElement = doc.RootElement();
    if (!rootElement) {
        throw std::runtime_error("XML file has no root element");
    }
    
    // Get scenario name
    const char* scenarioName = rootElement->Attribute("name");
    if (!scenarioName) {
        throw std::runtime_error("Scenario element missing 'name' attribute");
    }
    
    auto scenario = std::make_shared<Scenario>(scenarioName);
    
    // Parse Simulation section
    XMLElement* simElement = rootElement->FirstChildElement("Simulation");
    if (simElement) {
        scenario->config = parseSimulationConfig(simElement);
    }
    
    // Parse PlatformParameters section (optional)
    XMLElement* paramsElement = rootElement->FirstChildElement("PlatformParameters");
    if (paramsElement) {
        scenario->platformParams = parsePlatformParameters(paramsElement);
    }
    
    // Parse Events section
    XMLElement* eventsElement = rootElement->FirstChildElement("Events");
    if (eventsElement) {
        scenario->events = parseEvents(eventsElement);
        
        // Sort events by time
        std::sort(scenario->events.begin(), scenario->events.end(),
                  [](const ContactEvent& a, const ContactEvent& b) {
                      return a.time < b.time;
                  });
    }
    
    std::cout << "Loaded scenario: " << scenario->name << "\n";
    std::cout << "  Duration: " << scenario->config.duration << " s\n";
    std::cout << "  Timestep: " << scenario->config.timestep << " s\n";
    std::cout << "  Total events: " << scenario->events.size() << "\n";
    
    return scenario;
}

SimulationConfig ScenarioLoader::parseSimulationConfig(const XMLElement* simElement) {
    SimulationConfig config;
    
    // Parse duration
    const XMLElement* durationElement = simElement->FirstChildElement("duration");
    if (durationElement) {
        config.duration = durationElement->DoubleText();
    }
    
    // Parse timestep
    const XMLElement* timestepElement = simElement->FirstChildElement("timestep");
    if (timestepElement) {
        config.timestep = timestepElement->DoubleText();
    }
    
    return config;
}

PlatformParameters ScenarioLoader::parsePlatformParameters(const XMLElement* paramsElement) {
    PlatformParameters params;
    
    // Parse mass
    const XMLElement* massElement = paramsElement->FirstChildElement("mass");
    if (massElement) {
        params.mass = massElement->DoubleText();
    }
    
    // Parse inertia matrix
    const XMLElement* inertiaElement = paramsElement->FirstChildElement("inertia");
    if (inertiaElement) {
        // Expected format: Ixx, Iyy, Izz (diagonal matrix for a platform)
        const XMLElement* ixxElement = inertiaElement->FirstChildElement("Ixx");
        const XMLElement* iyyElement = inertiaElement->FirstChildElement("Iyy");
        const XMLElement* izzElement = inertiaElement->FirstChildElement("Izz");
        
        if (ixxElement) params.inertia(0, 0) = ixxElement->DoubleText();
        if (iyyElement) params.inertia(1, 1) = iyyElement->DoubleText();
        if (izzElement) params.inertia(2, 2) = izzElement->DoubleText();
    }
    
    return params;
}

std::vector<ContactEvent> ScenarioLoader::parseEvents(const XMLElement* eventsElement) {
    std::vector<ContactEvent> events;
    
    for (const XMLElement* contactElement = eventsElement->FirstChildElement("Contact");
         contactElement != nullptr;
         contactElement = contactElement->NextSiblingElement("Contact")) {
        
        ContactEvent event = parseContactEvent(contactElement);
        events.push_back(event);
    }
    
    return events;
}

ContactEvent ScenarioLoader::parseContactEvent(const XMLElement* contactElement) {
    ContactEvent event;
    
    // Get contact time
    const char* timeAttr = contactElement->Attribute("time");
    if (!timeAttr) {
        throw std::runtime_error("Contact element missing 'time' attribute");
    }
    event.time = std::stod(timeAttr);
    
    // Parse force readings
    for (const XMLElement* forceElement = contactElement->FirstChildElement("Force");
         forceElement != nullptr;
         forceElement = forceElement->NextSiblingElement("Force")) {
        
        ForceReading reading;
        
        // Get sensor ID
        const char* sensorAttr = forceElement->Attribute("sensor");
        if (!sensorAttr) {
            throw std::runtime_error("Force element missing 'sensor' attribute");
        }
        reading.sensorID = std::stoi(sensorAttr);
        
        // Get force components
        const char* fxAttr = forceElement->Attribute("fx");
        const char* fyAttr = forceElement->Attribute("fy");
        const char* fzAttr = forceElement->Attribute("fz");
        
        reading.fx = fxAttr ? std::stod(fxAttr) : 0.0;
        reading.fy = fyAttr ? std::stod(fyAttr) : 0.0;
        reading.fz = fzAttr ? std::stod(fzAttr) : 0.0;
        
        event.forces.push_back(reading);
    }
    
    return event;
}
